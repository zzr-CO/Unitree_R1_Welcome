/**
 * 1.0版本 动作添加奥特曼激光
=============================================================
 * Unitree R1 — 展厅接待程序（双音色版本）
 * Showroom Reception (PCM audio + DDS-based)
 * 
 * 按键操作（已按你的需求修改）：
 *   ← + A   → 话术1 + 奥特曼光线(24)（单次）
 *   ← + B   → 话术2 + 脸部挥手(25) + 右手比耶（单次）
 *   ← + X   → 话术3 + 手放胸口鞠躬(33)（单次）
 *   ← + Y   → 话术4 + 点赞/肯定(19) + 双手点赞（单次）
 *   ↓ + A   → 四场景循环模式 开始/停止（间隔%ds）
 *   SELECT  → 切换音色版本（原始版 ↔ 东北话版）
 *
 * PCM音频文件位于：/home/unitree/voice_pack/audio_show/
 *   - 原始版: morning_01.pcm ~ morning_04.pcm
 *   - 东北话版: morning_01_xiaobei.pcm ~ morning_04_xiaobei.pcm
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
 *
 * 右手灵巧手说明：
 *   - 程序只在需要手势或复位时，向 rt/brainco/right/cmd 短时间发布右手指令
 *   - 每个场景可配置动作前、动作中、动作后的灵巧手手势
 *   - ←+Y 场景会同时向左右手发布点赞姿态
 *   - 场景结束和程序退出时会发布张开复位 [0,0,0,0,0,0]
 *   - 平时不持续发布右手指令，避免和独立灵巧手测试程序抢控制权
 *   - 该功能依赖 brainco_hand_server 已启动并成功绑定右手
 *     启动命令：
 *       cd ~/brainco_hand_service/bin
 *       sudo ./brainco_hand_server --network_interface eth10
 * =============================================================
 */

#include <algorithm>
#include <array>
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

#include "unitree/idl/go2/MotorCmds_.hpp"
#include "unitree/idl/go2/WirelessController_.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/g1/arm/g1_arm_action_client.hpp"
#include "unitree/robot/g1/audio/g1_audio_client.hpp"

#define TOPIC_JOYSTICK "rt/wirelesscontroller"
#define TOPIC_LEFT_HAND "rt/brainco/left/cmd"
#define TOPIC_RIGHT_HAND "rt/brainco/right/cmd"

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

/* ==================== 音色版本枚举 ==================== */
enum VoiceVersion {
    VOICE_DEFAULT = 0,   // 原始音色
    VOICE_XIAOBEI = 1    // 东北话（Xiaobei）
};

static const char* VOICE_NAMES[] = {
    "原始音色",
    "东北话音色 (Xiaobei)"
};
/* ===================================================== */

static constexpr uint8_t VOLUME       = 100;
static constexpr int     ACT_RELEASE  = 99;
static constexpr int     COOLDOWN_MS  = 400;
static constexpr int     CHUNK_SIZE   = 32000;   // 成功案例的chunk size
static constexpr int     LOOP_GAP_SEC = 2;       // 循环模式间隔（秒）
static constexpr double  WAIT_BUFFER  = 1.0;     // 音频结束后等待1秒再接受新指令

static constexpr int PCM_RATE      = 16000;
static constexpr int PCM_CHANNELS  = 1;
static constexpr int PCM_BYTES_SEC = PCM_RATE * PCM_CHANNELS * 2;

/* ==================== 右手灵巧手 DDS 控制 ==================== */
using HandCmds = unitree_go::msg::dds_::MotorCmds_;
using HandPose = std::array<float, 6>;

static constexpr float HAND_FINGER_SPEED = 1.0f;
static constexpr int RIGHT_HAND_PUBLISH_INTERVAL_MS = 100;
static constexpr int RIGHT_HAND_PUBLISH_DURATION_MS = 1000;
static constexpr int RIGHT_HAND_PREPARE_MS = 500;

enum class HandGesture {
    None,
    Open,
    Relaxed,
    VSign,
    ThumbUp,
    OK,
    Point,
};

static const HandPose HAND_OPEN_POSE    = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
static const HandPose HAND_RELAXED_POSE = {0.2f, 0.2f, 0.25f, 0.25f, 0.25f, 0.25f};
static const HandPose HAND_V_POSE       = {1.f, 1.f, 0.f, 0.f, 1.f, 1.f};
static const HandPose HAND_THUMB_UP_POSE = {0.f, 0.f, 1.f, 1.f, 1.f, 1.f};
static const HandPose HAND_OK_POSE      = {0.7f, 0.7f, 0.65f, 0.f, 0.f, 0.f};
static const HandPose HAND_POINT_POSE   = {1.f, 1.f, 0.f, 1.f, 1.f, 1.f};

static const char* HandGestureName(HandGesture gesture) {
    switch (gesture) {
        case HandGesture::Open:    return "张开";
        case HandGesture::Relaxed: return "轻松自然手";
        case HandGesture::VSign:   return "比耶";
        case HandGesture::ThumbUp: return "点赞";
        case HandGesture::OK:      return "OK";
        case HandGesture::Point:   return "竖起食指";
        case HandGesture::None:    return "无";
    }
    return "未知";
}

static const HandPose* GetHandPose(HandGesture gesture) {
    switch (gesture) {
        case HandGesture::Open:    return &HAND_OPEN_POSE;
        case HandGesture::Relaxed: return &HAND_RELAXED_POSE;
        case HandGesture::VSign:   return &HAND_V_POSE;
        case HandGesture::ThumbUp: return &HAND_THUMB_UP_POSE;
        case HandGesture::OK:      return &HAND_OK_POSE;
        case HandGesture::Point:   return &HAND_POINT_POSE;
        case HandGesture::None:    return nullptr;
    }
    return nullptr;
}

static void SetHandPoseMsg(HandCmds& msg, const HandPose& pose) {
    msg.cmds().resize(6);
    for (size_t i = 0; i < pose.size(); ++i) {
        msg.cmds()[i].q() = pose[i];
        msg.cmds()[i].dq() = HAND_FINGER_SPEED;
    }
}

static void PublishRightHandPose(unitree::robot::ChannelPublisher<HandCmds>& publisher,
                                 const HandPose& pose,
                                 std::chrono::milliseconds duration) {
    HandCmds msg;
    SetHandPoseMsg(msg, pose);
    const int repeat_count = std::max(1, static_cast<int>(
        duration.count() / RIGHT_HAND_PUBLISH_INTERVAL_MS));
    for (int i = 0; i < repeat_count; ++i) {
        publisher.Write(msg, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(RIGHT_HAND_PUBLISH_INTERVAL_MS));
    }
}

static bool PublishHandGesture(unitree::robot::ChannelPublisher<HandCmds>& publisher,
                               const char* hand_name,
                               HandGesture gesture,
                               std::chrono::milliseconds duration) {
    const HandPose* pose = GetHandPose(gesture);
    if (!pose) return false;
    LOG("  发布" + std::string(hand_name) + "手势: " + std::string(HandGestureName(gesture))
        + " (" + std::to_string(duration.count()) + "ms)");
    PublishRightHandPose(publisher, *pose, duration);
    return true;
}

static int PublishHandGestures(unitree::robot::ChannelPublisher<HandCmds>& left_publisher,
                               unitree::robot::ChannelPublisher<HandCmds>& right_publisher,
                               HandGesture left_gesture,
                               HandGesture right_gesture,
                               std::chrono::milliseconds duration) {
    const HandPose* left_pose = GetHandPose(left_gesture);
    const HandPose* right_pose = GetHandPose(right_gesture);
    if (!left_pose && !right_pose) return 0;

    HandCmds left_msg;
    HandCmds right_msg;
    if (left_pose) SetHandPoseMsg(left_msg, *left_pose);
    if (right_pose) SetHandPoseMsg(right_msg, *right_pose);

    std::string log = "  发布灵巧手手势: ";
    if (left_pose) log += "左手=" + std::string(HandGestureName(left_gesture)) + " ";
    if (right_pose) log += "右手=" + std::string(HandGestureName(right_gesture)) + " ";
    log += "(" + std::to_string(duration.count()) + "ms)";
    LOG(log);

    const int repeat_count = std::max(1, static_cast<int>(
        duration.count() / RIGHT_HAND_PUBLISH_INTERVAL_MS));
    for (int i = 0; i < repeat_count; ++i) {
        if (left_pose) left_publisher.Write(left_msg, 0);
        if (right_pose) right_publisher.Write(right_msg, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(RIGHT_HAND_PUBLISH_INTERVAL_MS));
    }

    int mask = 0;
    if (left_pose) mask |= 1;
    if (right_pose) mask |= 2;
    return mask;
}
/* ============================================================= */

/* ==================== 4 个场景 ==================== */
static constexpr const char* PCM_DIR = "/home/unitree/voice_pack/audio_show/";

struct Scene {
    const char* pcm_file_default;   // 原始音色
    const char* pcm_file_xiaobei;   // 东北话音色
    int32_t     action_id;
    const char* action_name;
    const char* key_name;
    HandGesture start_gesture;      // 场景开始前短暂准备手势
    HandGesture during_gesture;     // 执行动作时配合手势
    HandGesture end_gesture;        // 场景结束后的停留手势
    int         hold_ms;            // 结束手势停留时间
    HandGesture left_start_gesture; // 左手：场景开始前短暂准备手势
    HandGesture left_during_gesture;// 左手：执行动作时配合手势
    HandGesture left_end_gesture;   // 左手：场景结束后的停留手势
    int         left_hold_ms;       // 左手：结束手势停留时间
};

static const Scene SCENES[] = {
    {"morning_01.pcm", "morning_01_xiaobei.pcm", 24, "奥特曼光线",    "←+A",
     HandGesture::None,    HandGesture::None,    HandGesture::None,       0,
     HandGesture::None,    HandGesture::None,    HandGesture::None,       0},
    {"morning_02.pcm", "morning_02_xiaobei.pcm", 25, "脸部挥手",      "←+B",
     HandGesture::Open,    HandGesture::VSign,   HandGesture::None,       0,
     HandGesture::None,    HandGesture::None,    HandGesture::None,       0},
    {"morning_03.pcm", "morning_03_xiaobei.pcm", 33, "手放胸口鞠躬",  "←+X",
     HandGesture::Relaxed, HandGesture::None,    HandGesture::OK,      1500,
     HandGesture::None,    HandGesture::None,    HandGesture::None,       0},
    {"morning_04.pcm", "morning_04_xiaobei.pcm", 19, "点赞/肯定+双手点赞", "←+Y",
     HandGesture::Open,    HandGesture::ThumbUp, HandGesture::ThumbUp, 1500,
     HandGesture::Open,    HandGesture::ThumbUp, HandGesture::ThumbUp, 1500},
};
static constexpr int SCENE_COUNT = sizeof(SCENES) / sizeof(SCENES[0]);
/* ===================================================== */

/* ==================== 动作时长表（实测数据，单位秒）==================== */
static double GetActionDuration(int32_t action_id) {
    switch (action_id) {
        case 99: return 0.306;   // release_arm
        case 11: return 4.726;   // blow_kiss_with_both_hands
        case 12: return 4.564;   // blow_kiss_with_left_hand
        case 13: return 4.566;   // blow_kiss_with_right_hand
        case 15: return 1.531;   // both_hands_up
        case 17: return 3.752;   // clamp
        case 18: return 1.629;   // high_five
        case 19: return 2.701;   // hug
        case 22: return 1.450;   // refuse
        case 23: return 1.560;   // right_hand_up
        case 24: return 2.632;   // ultraman_ray
        case 25: return 4.160;   // wave_under_head
        case 26: return 7.241;   // wave_above_head
        case 27: return 1.643;   // shake_hand
        case 28: return 1.519;   // box_left_hand_win
        case 29: return 1.542;   // box_right_hand_win
        case 30: return 3.042;   // box_both_hand_win
        case 31: return 1.552;   // extend_right_arm_forward
        case 33: return 2.129;   // right_hand_on_heart
        case 34: return 1.518;   // both_hands_up_deviate_right
        case 35: return 1.785;   // emphasize
        case 36: return 2.030;   // forward_push
        default: return 0.0;     // 未知动作，不额外等待
    }
}
/* ================================================================ */

static std::atomic<bool> g_running{true};
static std::atomic<bool> g_busy{false};
static std::atomic<bool> g_loop_on{false};  // ↓+A 循环模式

/* 音色版本控制 */
static std::atomic<VoiceVersion> g_voice_version{VOICE_DEFAULT};

/* 音频数据：每个场景有两套（原始版 + 东北话版） */
static std::vector<uint8_t> g_pcm_data[SCENE_COUNT][2];  // [scene_id][voice_version]
static double               g_pcm_duration[SCENE_COUNT][2] = {{0}};

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
               unitree::robot::ChannelPublisher<HandCmds>& left_hand_pub,
               unitree::robot::ChannelPublisher<HandCmds>& right_hand_pub,
               bool arm_ok)
{
    if (id < 0 || id >= SCENE_COUNT) return;
    
    VoiceVersion version = g_voice_version.load();
    const auto& s   = SCENES[id];
    auto&       pcm = g_pcm_data[id][version];
    double      dur = g_pcm_duration[id][version];
    const char* pcm_file = (version == VOICE_DEFAULT) ? s.pcm_file_default : s.pcm_file_xiaobei;
    
    if (pcm.empty()) {
        WARN("场景 " + std::to_string(id + 1) + " PCM 为空，跳过");
        return;
    }

    LOG("▶ 触发 " + std::string(s.key_name) + " [音色=" + VOICE_NAMES[version]
        + ", 音频=" + std::string(pcm_file)
        + ", 动作=" + s.action_name + "(id=" + std::to_string(s.action_id) + ")"
        + ", 右手=" + std::string(HandGestureName(s.during_gesture))
        + "/" + HandGestureName(s.end_gesture)
        + ", 左手=" + HandGestureName(s.left_during_gesture)
        + "/" + HandGestureName(s.left_end_gesture)
        + ", 时长=" + std::to_string(dur) + "s]");

    bool used_right_hand = false;
    bool used_left_hand = false;
    int hand_mask = PublishHandGestures(
        left_hand_pub, right_hand_pub,
        s.left_start_gesture, s.start_gesture,
        std::chrono::milliseconds(RIGHT_HAND_PREPARE_MS));
    used_left_hand = used_left_hand || ((hand_mask & 1) != 0);
    used_right_hand = used_right_hand || ((hand_mask & 2) != 0);

    /* 绿灯 */
    audio->LedControl(0, 255, 0);

    /* 发送 PCM 流 */
    std::ostringstream sid;
    sid << "show_" << id << "_" << version << "_"
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

    hand_mask = PublishHandGestures(
        left_hand_pub, right_hand_pub,
        s.left_during_gesture, s.during_gesture,
        std::chrono::milliseconds(RIGHT_HAND_PUBLISH_DURATION_MS));
    used_left_hand = used_left_hand || ((hand_mask & 1) != 0);
    used_right_hand = used_right_hand || ((hand_mask & 2) != 0);

    /* 触发手臂动作 */
    if (arm_ok) {
        int ret = arm->ExecuteAction(s.action_id);
        if (ret != 0) {
            WARN("  动作执行失败: " + std::string(s.action_name)
                 + " (id=" + std::to_string(s.action_id)
                 + ") 返回码=" + std::to_string(ret));
        }
    }

    /* 等待音频播完（动作已执行了 action_dur 秒，只需等剩余时间 + 缓冲）*/
    double action_dur = GetActionDuration(s.action_id);
    double remaining = std::max(0.0, dur - action_dur);  // 音频还剩多少
    int wait_ms = static_cast<int>(remaining * 1000) + static_cast<int>(WAIT_BUFFER * 1000);
    int waited = 0;
    while (waited < wait_ms && g_running.load()) {
        std::this_thread::sleep_for(100ms);
        waited += 100;
    }

    const int end_hold_ms = std::max(s.hold_ms, s.left_hold_ms);
    if (end_hold_ms > 0 && (s.end_gesture != HandGesture::None ||
                            s.left_end_gesture != HandGesture::None)) {
        hand_mask = PublishHandGestures(
            left_hand_pub, right_hand_pub,
            s.left_end_gesture, s.end_gesture,
            std::chrono::milliseconds(std::max(RIGHT_HAND_PUBLISH_INTERVAL_MS, end_hold_ms)));
        used_left_hand = used_left_hand || ((hand_mask & 1) != 0);
        used_right_hand = used_right_hand || ((hand_mask & 2) != 0);
    }

    /* 结束（先灭灯，再收尾，避免灯还亮着但什么都做不了的错觉） */
    audio->LedControl(0, 0, 0);
    audio->PlayStop("r1_show");
    if (arm_ok) arm->ExecuteAction(ACT_RELEASE);
    if (used_right_hand || used_left_hand) {
        PublishHandGestures(
            left_hand_pub, right_hand_pub,
            used_left_hand ? HandGesture::Open : HandGesture::None,
            used_right_hand ? HandGesture::Open : HandGesture::None,
            std::chrono::milliseconds(RIGHT_HAND_PUBLISH_DURATION_MS));
    }
    LOG("  ✓ 场景完成");
}

/* ========== 遥控器 ========== */
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
        select_btn.Update(key.bits.select);
    }
    bool DownA()  const { return down.on_press && a.pressed; }  // ↓+A 循环开关
    bool LeftA()  const { return left.on_press && a.pressed; }  // ←+A
    bool LeftB()  const { return left.on_press && b.pressed; }  // ←+B
    bool LeftX()  const { return left.on_press && x.pressed; }  // ←+X
    bool LeftY()  const { return left.on_press && y.pressed; }  // ←+Y
    bool SelectPress() const { return select_btn.on_press; }     // SELECT 切换音色
    
    struct Btn { bool pressed=false, on_press=false;
        void Update(bool s) { on_press=s&&!pressed; pressed=s; }
    };
    Btn down, left, a, b, x, y, select_btn;
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
    LOG("══════ R1 展厅接待程序启动（双音色版本）══════");
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

    /* 灵巧手 DDS Publisher */
    unitree::robot::ChannelPublisher<HandCmds> left_hand_pub(TOPIC_LEFT_HAND);
    left_hand_pub.InitChannel();
    LOG("左手灵巧手 DDS 按需发布就绪: " + std::string(TOPIC_LEFT_HAND));

    unitree::robot::ChannelPublisher<HandCmds> right_hand_pub(TOPIC_RIGHT_HAND);
    right_hand_pub.InitChannel();
    LOG("右手灵巧手 DDS 按需发布就绪: " + std::string(TOPIC_RIGHT_HAND));

    /* PCM - 加载两套音频 */
    LOG("加载 PCM 音频（双音色版本）...");
    int loaded_default = 0, loaded_xiaobei = 0;
    
    for (int i = 0; i < SCENE_COUNT; ++i) {
        // 加载原始音色
        std::string path_default = std::string(PCM_DIR) + SCENES[i].pcm_file_default;
        if (LoadPcm(path_default, g_pcm_data[i][VOICE_DEFAULT], g_pcm_duration[i][VOICE_DEFAULT])) {
            LOG("  ✅ [原始] " + std::string(SCENES[i].pcm_file_default)
                + " (" + std::to_string(g_pcm_data[i][VOICE_DEFAULT].size()/1024) + "KB, "
                + std::to_string(g_pcm_duration[i][VOICE_DEFAULT]) + "s)");
            loaded_default++;
        } else {
            ERR("  ❌ [原始] " + std::string(SCENES[i].pcm_file_default) + " 未找到");
        }
        
        // 加载东北话音色
        std::string path_xiaobei = std::string(PCM_DIR) + SCENES[i].pcm_file_xiaobei;
        if (LoadPcm(path_xiaobei, g_pcm_data[i][VOICE_XIAOBEI], g_pcm_duration[i][VOICE_XIAOBEI])) {
            LOG("  ✅ [东北话] " + std::string(SCENES[i].pcm_file_xiaobei)
                + " (" + std::to_string(g_pcm_data[i][VOICE_XIAOBEI].size()/1024) + "KB, "
                + std::to_string(g_pcm_duration[i][VOICE_XIAOBEI]) + "s)");
            loaded_xiaobei++;
        } else {
            WARN("  ⚠️ [东北话] " + std::string(SCENES[i].pcm_file_xiaobei) + " 未找到");
        }
    }
    
    LOG("加载完成: 原始音色 " + std::to_string(loaded_default) + "/" + std::to_string(SCENE_COUNT)
        + ", 东北话音色 " + std::to_string(loaded_xiaobei) + "/" + std::to_string(SCENE_COUNT));

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
              << "  Unitree R1 — 展厅接待控制（双音色版本）\n"
              << "  当前音色: " << VOICE_NAMES[g_voice_version.load()] << "\n"
              << "  按键控制:\n";
    for (int i = 0; i < SCENE_COUNT; ++i)
        std::cout << "    " << SCENES[i].key_name << " → "
                  << SCENES[i].action_name
                  << " / 右手: " << HandGestureName(SCENES[i].during_gesture)
                  << "→" << HandGestureName(SCENES[i].end_gesture)
                  << " / 左手: " << HandGestureName(SCENES[i].left_during_gesture)
                  << "→" << HandGestureName(SCENES[i].left_end_gesture)
                  << " (单次)\n";
    std::cout << "    ↓+A → 四场景循环 (间隔 " << LOOP_GAP_SEC << "s, 按一次开再按关)\n"
              << "    SELECT → 切换音色 (" << VOICE_NAMES[0] << " ↔ " << VOICE_NAMES[1] << ")\n"
              << "============================================================\n\n";

    /* ==================== 主循环 ==================== */
    while (g_running.load()) {
        { std::lock_guard<std::mutex> lk(mx); gp.Update(msg); }

        /* ── SELECT: 切换音色版本 ── */
        if (gp.SelectPress()) {
            VoiceVersion current = g_voice_version.load();
            VoiceVersion next = (current == VOICE_DEFAULT) ? VOICE_XIAOBEI : VOICE_DEFAULT;
            g_voice_version.store(next);
            LOG("[SELECT] 切换音色: " + std::string(VOICE_NAMES[current]) 
                + " → " + std::string(VOICE_NAMES[next]));
            std::cout << "\n🔊 当前音色: " << VOICE_NAMES[next] << "\n\n";
            std::this_thread::sleep_for(400ms);
        }

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

            if (scene_id >= 0 && !g_busy.load() && !g_pcm_data[scene_id][g_voice_version.load()].empty()) {
                g_busy.store(true);
                PlayScene(scene_id, audio, arm, left_hand_pub, right_hand_pub, arm_ok);
                g_busy.store(false);
                std::this_thread::sleep_for(COOLDOWN_MS * 1ms);
            }
        }

        /* ── 循环模式 ── */
        if (g_loop_on.load() && !g_busy.load()) {
            g_busy.store(true);
            for (int i = 0; i < SCENE_COUNT && g_loop_on.load() && g_running.load(); ++i) {
                if (g_pcm_data[i][g_voice_version.load()].empty()) continue;
                PlayScene(i, audio, arm, left_hand_pub, right_hand_pub, arm_ok);

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
                        if (gp.SelectPress()) {
                            VoiceVersion current = g_voice_version.load();
                            VoiceVersion next = (current == VOICE_DEFAULT) ? VOICE_XIAOBEI : VOICE_DEFAULT;
                            g_voice_version.store(next);
                            LOG("[SELECT] 循环模式中切换音色: " + std::string(VOICE_NAMES[current]) 
                                + " → " + std::string(VOICE_NAMES[next]));
                            std::this_thread::sleep_for(400ms);
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
    LOG("发布双手张开复位");
    PublishHandGestures(left_hand_pub, right_hand_pub,
                        HandGesture::Open, HandGesture::Open,
                        std::chrono::milliseconds(RIGHT_HAND_PUBLISH_DURATION_MS));
    audio->LedControl(0, 0, 0);
    LOG("程序已退出");
    return 0;
}
