/**
 * =============================================================
 * Unitree R1 — 展厅接待程序（修改自 r1_remote_ctrl.cpp 成功案例）
 * Showroom Reception (PCM audio + DDS-based)
 *
 * 按键操作（已按你的需求修改）：
 *   ← + A   → 话术1 + 高举挥手(26)（单次）
 *   ← + B   → 话术2 + 脸部挥手(25)（单次）
 *   ← + X   → 话术3 + 手放胸口鞠躬(33)（单次）
 *   ← + Y   → 话术4 + 右手举起示意(23)（单次）
 *   ↓ + A   → 四场景循环模式 开始/停止（间隔3秒）
 *
 * PCM音频文件位于：/home/unitree/voice_pack/audio_show/
 *   - show_01.pcm ~ show_04.pcm
 *
 * 日志位置：
 *   实时查看: sudo journalctl -u r1-show.service -f
 *
 * 编译（在 unitree_sdk2-1.0 根目录）：
 *   g++ -std=c++17 r1_show_control.cpp \
 *       -I./include -I./thirdparty/include/ddscxx \
 *       -L./lib/aarch64 -L./thirdparty/lib/aarch64 \
 *       -lunitree_sdk2 -lddscxx -lddsc -lpthread \
 *       -Wl,-rpath,./lib/aarch64:./thirdparty/lib/aarch64 \
 *       -o r1_show_control
 *
 * 运行：
 *   ./r1_show_control eth10
 * =============================================================
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <sys/stat.h>

#include "unitree/idl/go2/WirelessController_.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/g1/arm/g1_arm_action_client.hpp"
#include "unitree/robot/g1/audio/g1_audio_client.hpp"

#define TOPIC_JOYSTICK "rt/wirelesscontroller"

using namespace std::chrono_literals;

/* ============ 时间戳日志（同时输出到 stdout 和本地文件）============ */
static constexpr const char* LOG_FILE = "/var/log/r1_show_control.log";
static std::mutex g_log_mutex;

static std::string NowStr() {
    auto t = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(t);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        t.time_since_epoch()) % 1000;
    std::ostringstream ss;
    ss << std::put_time(std::localtime(&tt), "%H:%M:%S")
       << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

static void WriteLog(const std::string& line) {
    std::lock_guard<std::mutex> lk(g_log_mutex);
    std::cout << line << std::endl;
    std::ofstream f(LOG_FILE, std::ios::app);
    if (f) f << line << "\n";
}

#define LOG(msg)  WriteLog("[" + NowStr() + "] " + std::string(msg))
#define ERR(msg)  WriteLog("[" + NowStr() + "] [ERROR] " + std::string(msg))
#define WARN(msg) WriteLog("[" + NowStr() + "] [WARN] " + std::string(msg))
/* ===================================== */

/* ==================== 4 个场景（已替换为你的需求）==================== */
static constexpr const char* PCM_DIR = "/home/unitree/voice_pack/audio_show/";

struct Scene {
    const char* pcm_file;
    int32_t     action_id;
    const char* action_name;
    const char* key_name;
};

static const Scene SCENES[] = {
    {"morning_01_xiaobei.pcm", 26, "高举挥手",      "←+A"},
    {"morning_02_xiaobei.pcm", 25, "脸部挥手",      "←+B"},
    {"morning_03_xiaobei.pcm", 33, "手放胸口鞠躬",  "←+X"},
    {"morning_04_xiaobei.pcm", 23, "右手举起示意",  "←+Y"},
};
static constexpr int SCENE_COUNT = sizeof(SCENES) / sizeof(SCENES[0]);
/* =============================================================== */

static constexpr uint8_t VOLUME       = 100;
static constexpr int     ACT_RELEASE  = 99;
static constexpr int     COOLDOWN_MS  = 400;
static constexpr int     CHUNK_SIZE   = 32000;   // 成功案例的chunk size
static constexpr int     LOOP_GAP_SEC = 2;       // 循环模式间隔（秒）

static constexpr int PCM_RATE      = 16000;
static constexpr int PCM_CHANNELS  = 1;
static constexpr int PCM_BYTES_SEC = PCM_RATE * PCM_CHANNELS * 2;

static std::atomic<bool> g_running{true};
static std::atomic<bool> g_busy{false};
static std::atomic<bool> g_loop_on{false};  // ↓+A 循环模式

static std::vector<uint8_t> g_pcm_data[SCENE_COUNT];
static double               g_pcm_duration[SCENE_COUNT] = {0};

void SignalHandler(int) {
    LOG("收到退出信号");
    g_running.store(false);
    g_loop_on.store(false);
}

bool LoadPcm(const std::string& path, std::vector<uint8_t>& out, double& dur) {
    struct stat st;
    if (stat(path.c_str(), &st) != 0) {
        ERR("找不到 PCM 文件: " + path);
        return false;
    }
    std::ifstream f(path, std::ios::binary);
    if (!f) { ERR("无法打开: " + path); return false; }
    out.resize(st.st_size);
    f.read(reinterpret_cast<char*>(out.data()), st.st_size);
    dur = static_cast<double>(st.st_size) / PCM_BYTES_SEC;
    return true;
}

/** 执行单次场景：播放 PCM + 触发动作 */
void PlayScene(int id,
               std::shared_ptr<unitree::robot::g1::AudioClient> audio,
               std::shared_ptr<unitree::robot::g1::G1ArmActionClient> arm,
               bool arm_ok)
{
    if (id < 0 || id >= SCENE_COUNT) return;
    if (g_pcm_data[id].empty()) {
        WARN("场景 " + std::to_string(id + 1) + " PCM 为空，跳过");
        return;
    }

    const auto& s   = SCENES[id];
    auto&       pcm = g_pcm_data[id];
    double      dur = g_pcm_duration[id];

    LOG("▶ 触发 " + std::string(s.key_name) + " [音频=" + s.pcm_file
        + ", 动作=" + s.action_name + "(id=" + std::to_string(s.action_id) + ")"
        + ", 时长=" + std::to_string(dur) + "s]");

    /* 绿灯 */
    audio->LedControl(0, 255, 0);

    /* 发送 PCM 流 */
    std::ostringstream sid;
    sid << "show_" << id << "_"
        << std::chrono::steady_clock::now().time_since_epoch().count();
    std::string stream_id = sid.str();

    size_t total  = pcm.size();
    size_t offset = 0;
    while (offset < total) {
        size_t n = std::min(static_cast<size_t>(CHUNK_SIZE), total - offset);
        std::vector<uint8_t> chunk(pcm.begin() + offset, pcm.begin() + offset + n);
        audio->PlayStream("r1_show", stream_id, chunk);
        offset += n;
        std::this_thread::sleep_for(100ms);
    }
    LOG("  PCM 发送完毕 (" + std::to_string(total / 1024) + "KB)");

    /* 触发手臂动作 */
    if (arm_ok) {
        int ret = arm->ExecuteAction(s.action_id);
        if (ret != 0) {
            WARN("  动作执行失败: " + std::string(s.action_name)
                 + " (id=" + std::to_string(s.action_id)
                 + ") 返回码=" + std::to_string(ret));
        }
    }

    /* 等待音频播完 */
    int wait_ms = static_cast<int>(dur * 1000) + 500;
    int waited = 0;
    while (waited < wait_ms && g_running.load()) {
        std::this_thread::sleep_for(100ms);
        waited += 100;
    }

    /* 结束 */
    audio->PlayStop("r1_show");
    if (arm_ok) arm->ExecuteAction(ACT_RELEASE);
    audio->LedControl(0, 0, 0);

    LOG("  ✓ 场景完成");
}

/* ========== 遥控器（已按你的需求修改）========== */
union KeyUnion {
    struct {
        uint8_t R1:1, L1:1, start:1, select:1, R2:1, L2:1, F1:1, F2:1;
        uint8_t A:1, B:1, X:1, Y:1, up:1, right:1, down:1, left:1;
    } bits;
    uint16_t raw;
};

class Gamepad {
public:
    void Update(const unitree_go::msg::dds_::WirelessController_& msg) {
        key.raw = msg.keys();
        down.Update(key.bits.down);
        left.Update(key.bits.left);
        a.Update(key.bits.A); b.Update(key.bits.B);
        x.Update(key.bits.X); y.Update(key.bits.Y);
    }
    bool DownA()  const { return down.on_press && a.pressed; }  // ↓+A 循环开关
    bool LeftA()  const { return left.on_press && a.pressed; }  // ←+A
    bool LeftB()  const { return left.on_press && b.pressed; }  // ←+B
    bool LeftX()  const { return left.on_press && x.pressed; }  // ←+X
    bool LeftY()  const { return left.on_press && y.pressed; }  // ←+Y

    struct Btn { bool pressed=false, on_press=false;
        void Update(bool s) { on_press=s&&!pressed; pressed=s; }
    };
    Btn down, left, a, b, x, y;
private:
    KeyUnion key;
};
/* =================================================== */

int main(int argc, char const* argv[]) {
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <网络接口>\n";
        return 1;
    }

    std::string network = argv[1];
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    /* 清空旧日志，开始新会话 */
    { std::ofstream f(LOG_FILE, std::ios::trunc); }
    LOG("══════ R1 展厅接待程序启动 ═════");
    LOG("网络接口: " + network);

    /* DDS */
    unitree::robot::ChannelFactory::Instance()->Init(0, network.c_str());
    LOG("DDS 初始化完成");

    /* 语音 */
    auto audio = std::make_shared<unitree::robot::g1::AudioClient>();
    try {
        audio->Init();
        audio->SetTimeout(5.0f);
        audio->SetVolume(VOLUME);
        LOG("语音服务就绪 (音量=" + std::to_string(VOLUME) + ")");
    } catch (...) { ERR("语音服务初始化失败"); return 1; }

    /* 手臂 */
    auto arm = std::make_shared<unitree::robot::g1::G1ArmActionClient>();
    bool arm_ok = false;
    try {
        arm->Init();
        arm->SetTimeout(10.0f);
        std::this_thread::sleep_for(500ms);
        LOG("手臂服务就绪");
        arm_ok = true;
    } catch (...) { WARN("手臂服务不可用，仅语音"); }

    /* PCM */
    LOG("加载 PCM 音频...");
    for (int i = 0; i < SCENE_COUNT; ++i) {
        std::string path = std::string(PCM_DIR) + SCENES[i].pcm_file;
        if (LoadPcm(path, g_pcm_data[i], g_pcm_duration[i])) {
            LOG("  ✅ " + std::string(SCENES[i].pcm_file)
                + " (" + std::to_string(g_pcm_data[i].size()/1024) + "KB, "
                + std::to_string(g_pcm_duration[i]) + "s)");
        } else {
            ERR("  ❌ " + std::string(SCENES[i].pcm_file) + " 未找到");
        }
    }

    /* 遥控器 */
    Gamepad gp;
    unitree_go::msg::dds_::WirelessController_ msg;
    std::mutex mx;
    auto sub = unitree::robot::ChannelSubscriber<
        unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK);
    sub.InitChannel([&](const void* m) {
        std::lock_guard<std::mutex> lk(mx);
        msg = *(unitree_go::msg::dds_::WirelessController_*)m;
    });
    LOG("遥控器监听就绪");

    /* 欢迎信息 */
    std::cout << "\n============================================================\n"
              << "  Unitree R1 — 展厅接待控制\n"
              << "  按键控制:\n";
    for (int i = 0; i < SCENE_COUNT; ++i)
        std::cout << "    " << SCENES[i].key_name << " → "
                  << SCENES[i].action_name << " (单次)\n";
    std::cout << "    ↓+A → 四场景循环 (间隔 " << LOOP_GAP_SEC << "s, 按一次开再按关)\n"
              << "============================================================\n\n";

    /* ==================== 主循环 ==================== */
    while (g_running.load()) {
        { std::lock_guard<std::mutex> lk(mx); gp.Update(msg); }

        /* ── ↓+A: 循环模式切换 ── */
        if (gp.DownA()) {
            if (g_loop_on.load()) {
                g_loop_on.store(false);
                LOG("[↓+A] ⏸ 循环模式 停止");
            } else {
                g_loop_on.store(true);
                LOG("[↓+A] ▶ 循环模式 开始 (4场景,间隔" + std::to_string(LOOP_GAP_SEC) + "s)");
            }
            std::this_thread::sleep_for(400ms);
        }

        /* ── 单键模式 ── */
        if (!g_loop_on.load()) {
            int scene_id = -1;
            if (gp.LeftA())      scene_id = 0;
            else if (gp.LeftB()) scene_id = 1;
            else if (gp.LeftX()) scene_id = 2;
            else if (gp.LeftY()) scene_id = 3;

            if (scene_id >= 0 && !g_busy.load() && !g_pcm_data[scene_id].empty()) {
                g_busy.store(true);
                PlayScene(scene_id, audio, arm, arm_ok);
                g_busy.store(false);
                std::this_thread::sleep_for(COOLDOWN_MS * 1ms);
            }
        }

        /* ── 循环模式 ── */
        if (g_loop_on.load() && !g_busy.load()) {
            g_busy.store(true);
            for (int i = 0; i < SCENE_COUNT && g_loop_on.load() && g_running.load(); ++i) {
                if (g_pcm_data[i].empty()) continue;
                PlayScene(i, audio, arm, arm_ok);

                /* 间隔（期间检查停止按键） */
                if (i < SCENE_COUNT - 1 && g_loop_on.load()) {
                    LOG("  ⏸ 等待 " + std::to_string(LOOP_GAP_SEC) + "s...");
                    int waited = 0;
                    while (waited < LOOP_GAP_SEC * 1000 && g_running.load() && g_loop_on.load()) {
                        std::this_thread::sleep_for(100ms);
                        waited += 100;
                        { std::lock_guard<std::mutex> lk(mx); gp.Update(msg); }
                        if (gp.DownA()) {
                            g_loop_on.store(false);
                            LOG("[↓+A] ⏸ 循环模式 停止");
                            std::this_thread::sleep_for(400ms);
                            break;
                        }
                    }
                }
            }
            g_busy.store(false);
            std::this_thread::sleep_for(COOLDOWN_MS * 1ms);
        }

        std::this_thread::sleep_for(50ms);
    }

    /* 退出 */
    LOG("安全退出中...");
    if (arm_ok) arm->ExecuteAction(ACT_RELEASE);
    audio->LedControl(0, 0, 0);
    LOG("程序已退出");
    return 0;
}
