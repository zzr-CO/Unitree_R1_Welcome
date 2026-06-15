/**
 * Unitree R1 自动早安问好程序
 *
 * 功能:
 *   1. 程序启动后先待机，不自动识别人。
 *   2. 按遥控器 ↓+X 后，通过 R1 videohub 获取摄像头 JPG 图片并调用 YOLO 检测人物。
 *   3. 连续检测到符合迎宾距离的人后，随机执行 4 套早安问好场景之一。
 *   4. 再次按遥控器 ↓+X 后停止识别；场景执行期间暂停检测。
 *
 * 安全边界:
 *   - 不发布 rt/lowcmd。
 *   - 不写任何底层电机角度。
 *   - R1 本体动作只调用已验证的官方 ExecuteAction(): 24, 25, 33, 19, 99。
 *   - 灵巧手只向 brainco_hand_server 的 DDS topic 发布已验证手势。
 *
 * 你可以改:
 *   - kDetectEveryMs: 检测间隔。
 *   - kNeedConsecutiveYes: 连续检测成功几次后触发。
 *   - kYoloConf / kMinHeightRatio / kCenterBand: YOLO 触发阈值。
 *
 * 暂时不要改:
 *   - TOPIC_LEFT_HAND / TOPIC_RIGHT_HAND。
 *   - 场景 action_id。
 *   - PlayScene() 里的语音、手臂、灵巧手执行顺序。
 *
 * 编译:
 *   cd ~/unitree_sdk2-2.0
 *   g++ -std=c++17 r1_morning_auto_greet.cpp \
 *       -I./include \
 *       -I./thirdparty/include \
 *       -I./thirdparty/include/ddscxx \
 *       -L./lib/aarch64 \
 *       -L./thirdparty/lib/aarch64 \
 *       -lunitree_sdk2 -lddscxx -lddsc -lpthread \
 *       -Wl,-rpath,./lib/aarch64:./thirdparty/lib/aarch64 \
 *       -o r1_morning_auto_greet
 *
 * 运行:
 *   ./r1_morning_auto_greet eth10
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>

#include "unitree/idl/go2/MotorCmds_.hpp"
#include "unitree/idl/go2/WirelessController_.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/g1/arm/g1_arm_action_client.hpp"
#include "unitree/robot/g1/audio/g1_audio_client.hpp"
#include "unitree/robot/go2/video/video_client.hpp"

#define TOPIC_JOYSTICK "rt/wirelesscontroller"
#define TOPIC_LEFT_HAND "rt/brainco/left/cmd"
#define TOPIC_RIGHT_HAND "rt/brainco/right/cmd"

using namespace std::chrono_literals;

// 你可以改: 自动检测相关参数。
static constexpr int kDetectEveryMs = 500; //500ms检测一次
static constexpr int kNeedConsecutiveYes = 2;  // 连续 2 次检测到符合条件的人后触发动作。
static constexpr double kYoloConf = 0.35;   // YOLO 置信度阈值，越大越严格。可以适当调小以增加触发概率，但可能增加误触。
static constexpr double kMinHeightRatio = 0.15;  // YOLO 检测框最小高度占比，越大越严格。可以适当调小以增加触发概率，但可能增加误触。
static constexpr double kCenterBand = 0.50;  // 中心带宽，0.5 表示检测框中心必须在画面中间 50% 的范围内。可以适当调大以增加触发概率，但可能增加误触。

// 你可以改: YOLO 文件路径。默认要求脚本和模型都放在 ~/unitree_sdk2-2.0。
static constexpr const char* kYoloScript = "yolo_person_detect_onnx.py";
static constexpr const char* kYoloModel = "yolov8n.onnx";
static constexpr const char* kDetectFramePath = "/tmp/r1_auto_greet_frame.jpg";
static constexpr const char* kDetectResultPath = "/tmp/r1_auto_greet_detect.jpg";

// 暂时不要改: 语音文件和日志路径。
static constexpr const char* kPcmDir = "/home/unitree/voice_pack/audio_show/";
static constexpr const char* kLogFile = "/var/log/r1_morning_auto_greet.log";

static constexpr uint8_t kVolume = 100; // 音量 0-100
static constexpr int kActRelease = 99;  // 官方 ExecuteAction 99 是放松动作，可以在动作结束后调用以确保安全。
static constexpr int kChunkSize = 32000;  // PCM 分块发送大小，单位字节。16000 采样率 * 1 通道 * 2 字节/采样 * 1 秒 = 32000 字节。
static constexpr double kWaitBufferSec = 1.0;
static constexpr int kPcmRate = 16000;
static constexpr int kPcmChannels = 1;
static constexpr int kPcmBytesPerSec = kPcmRate * kPcmChannels * 2;

static constexpr float kHandFingerSpeed = 1.0f;  // 灵巧手指关节速度，单位是关节全行程/秒。可以适当调大以加快手势变化，但可能增加机械磨损。
static constexpr int kHandPublishIntervalMs = 100;
static constexpr int kHandPublishDurationMs = 1000;
static constexpr int kHandPrepareMs = 500;

static std::atomic<bool> g_running{true};
static std::atomic<bool> g_vision_enabled{false};

//生成当前时间字符串
static std::string NowStr() {
    auto now = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::ostringstream ss;
    ss << std::put_time(std::localtime(&tt), "%H:%M:%S")
       << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

// 写日志
static void WriteLog(const std::string& line) {
    std::cout << line << std::endl;
    std::ofstream f(kLogFile, std::ios::app);
    if (f) {
        f << line << "\n";
    }
}

#define LOG(msg)  WriteLog("[" + NowStr() + "] " + std::string(msg))
#define WARN(msg) WriteLog("[" + NowStr() + "] [WARN] " + std::string(msg))
#define ERR(msg)  WriteLog("[" + NowStr() + "] [ERROR] " + std::string(msg))

void SignalHandler(int) {
    g_running.store(false);
    g_vision_enabled.store(false);
    LOG("收到退出信号，准备安全退出");
}

static void SetVisionStatusLed(std::shared_ptr<unitree::robot::g1::AudioClient> audio) {
    // 你可以改: 状态灯颜色。这里约定 RGB = (红, 绿, 蓝)。
    // 只在按键切换识别状态时设置灯色：开始识别蓝色，停止识别红色；平时不主动改默认灯。
    if (g_vision_enabled.load()) {
        audio->LedControl(0, 0, 255);
    } else {
        audio->LedControl(255, 0, 0);
    }
}

// 遥控器按键位。这个布局来自 Unitree WirelessController_ 的 keys() 字段。
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
        x.Update(key.bits.X);
    }

    // 你可以改: 如果以后想换成别的按键组合，只改这里的判断。
    // 这里写成两种顺序都能触发: 先按 ↓ 再按 X，或先按 X 再按 ↓。
    bool DownX() const {
        return (down.on_press && x.pressed) || (x.on_press && down.pressed);
    }

    struct Btn {
        bool pressed = false;
        bool on_press = false;
        void Update(bool s) {
            on_press = s && !pressed;
            pressed = s;
        }
    };

private:
    KeyUnion key{};
    Btn down;
    Btn x;
};

// 灵巧手手势: 6 个值对应 [thumb, thumb_aux, index, middle, ring, pinky]。
using HandCmds = unitree_go::msg::dds_::MotorCmds_;
using HandPose = std::array<float, 6>;

enum class HandGesture {
    None,
    Open,
    Relaxed,
    VSign,
    ThumbUp,
    OK,
};

static const HandPose kHandOpenPose = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
static const HandPose kHandRelaxedPose = {0.2f, 0.2f, 0.25f, 0.25f, 0.25f, 0.25f};
static const HandPose kHandVPose = {1.f, 1.f, 0.f, 0.f, 1.f, 1.f};
static const HandPose kHandThumbUpPose = {0.f, 0.f, 1.f, 1.f, 1.f, 1.f};
static const HandPose kHandOkPose = {0.7f, 0.7f, 0.65f, 0.f, 0.f, 0.f};

static const char* HandGestureName(HandGesture gesture) {
    switch (gesture) {
        case HandGesture::Open: return "张开";
        case HandGesture::Relaxed: return "轻松自然手";
        case HandGesture::VSign: return "比耶";
        case HandGesture::ThumbUp: return "点赞";
        case HandGesture::OK: return "OK";
        case HandGesture::None: return "无";
    }
    return "未知";
}

static const HandPose* GetHandPose(HandGesture gesture) {
    switch (gesture) {
        case HandGesture::Open: return &kHandOpenPose;
        case HandGesture::Relaxed: return &kHandRelaxedPose;
        case HandGesture::VSign: return &kHandVPose;
        case HandGesture::ThumbUp: return &kHandThumbUpPose;
        case HandGesture::OK: return &kHandOkPose;
        case HandGesture::None: return nullptr;
    }
    return nullptr;
}

// 把数组填进 DDS 消息
static void SetHandPoseMsg(HandCmds& msg, const HandPose& pose) {
    msg.cmds().resize(6);
    for (size_t i = 0; i < pose.size(); ++i) {
        msg.cmds()[i].q() = pose[i];
        msg.cmds()[i].dq() = kHandFingerSpeed;
    }
}

// 发布左右手手势
static int PublishHandGestures(unitree::robot::ChannelPublisher<HandCmds>& left_pub,
                               unitree::robot::ChannelPublisher<HandCmds>& right_pub,
                               HandGesture left_gesture,
                               HandGesture right_gesture,
                               std::chrono::milliseconds duration) {
    const HandPose* left_pose = GetHandPose(left_gesture);
    const HandPose* right_pose = GetHandPose(right_gesture);
    if (!left_pose && !right_pose) {
        return 0;
    }

    HandCmds left_msg;
    HandCmds right_msg;
    if (left_pose) SetHandPoseMsg(left_msg, *left_pose);
    if (right_pose) SetHandPoseMsg(right_msg, *right_pose);

    std::string log = "  发布灵巧手: ";
    if (left_pose) log += "左手=" + std::string(HandGestureName(left_gesture)) + " ";
    if (right_pose) log += "右手=" + std::string(HandGestureName(right_gesture)) + " ";
    log += "(" + std::to_string(duration.count()) + "ms)";
    LOG(log);

    const int repeat_count = std::max(1, static_cast<int>(
        duration.count() / kHandPublishIntervalMs));

    for (int i = 0; i < repeat_count && g_running.load(); ++i) {
        if (left_pose) left_pub.Write(left_msg, 0);
        if (right_pose) right_pub.Write(right_msg, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(kHandPublishIntervalMs));
    }

    int mask = 0;
    if (left_pose) mask |= 1;
    if (right_pose) mask |= 2;
    return mask;
}

struct Scene {
    const char* pcm_file;
    int32_t action_id;
    const char* action_name;
    HandGesture right_start;
    HandGesture right_during;
    HandGesture right_end;
    int right_hold_ms;
    HandGesture left_start;
    HandGesture left_during;
    HandGesture left_end;
    int left_hold_ms;
};

// 暂时不要改: 这四套动作都是项目里之前验证过的早安问好动作组合。
static const Scene kScenes[] = {
    {"morning_01.pcm", 24, "奥特曼光线",
     HandGesture::None, HandGesture::None, HandGesture::None, 0,
     HandGesture::None, HandGesture::None, HandGesture::None, 0},
    {"morning_02.pcm", 25, "脸部挥手+右手比耶",
     HandGesture::Open, HandGesture::VSign, HandGesture::None, 0,
     HandGesture::None, HandGesture::None, HandGesture::None, 0},
    {"morning_03.pcm", 33, "手放胸口鞠躬+OK",
     HandGesture::Relaxed, HandGesture::None, HandGesture::OK, 1500,
     HandGesture::None, HandGesture::None, HandGesture::None, 0},
    {"morning_04.pcm", 19, "点赞/肯定+双手点赞",
     HandGesture::Open, HandGesture::ThumbUp, HandGesture::ThumbUp, 1500,
     HandGesture::Open, HandGesture::ThumbUp, HandGesture::ThumbUp, 1500},
};
static constexpr int kSceneCount = sizeof(kScenes) / sizeof(kScenes[0]);

static std::vector<uint8_t> g_pcm_data[kSceneCount];
static double g_pcm_duration[kSceneCount] = {0.0};

static double GetActionDuration(int32_t action_id) {
    switch (action_id) {
        case 19: return 2.701;
        case 24: return 2.632;
        case 25: return 4.160;
        case 33: return 2.129;
        case 99: return 0.306;
        default: return 0.0;
    }
}

static bool LoadPcm(const std::string& path, std::vector<uint8_t>& out, double& duration) {
    struct stat st;
    if (stat(path.c_str(), &st) != 0) {
        ERR("找不到 PCM 文件: " + path);
        return false;
    }

    std::ifstream f(path, std::ios::binary);
    if (!f) {
        ERR("无法打开 PCM 文件: " + path);
        return false;
    }

    out.resize(st.st_size);
    f.read(reinterpret_cast<char*>(out.data()), st.st_size);
    duration = static_cast<double>(st.st_size) / kPcmBytesPerSec;
    return true;
}

static bool SaveJpg(const std::vector<uint8_t>& image, const std::string& path) {
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        return false;
    }
    f.write(reinterpret_cast<const char*>(image.data()), image.size()); // 写入 JPG 数据
    return static_cast<bool>(f);
}

static std::string RunCommandCapture(const std::string& command) {
    std::string output;
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        return output;
    }

    char buffer[512];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        output += buffer;
    }
    pclose(pipe);
    return output;
}

static bool DetectPersonInFrame() {
    // 你应该学会: C++ 程序负责取视频流，Python 脚本负责跑 YOLO。
    std::ostringstream cmd; // 注意：命令末尾的 "2>&1" 是为了捕获 Python 脚本的标准错误输出，以便在日志中记录可能的错误信息。
    cmd << "python3 " << kYoloScript  
        << " " << kDetectFramePath
        << " --model " << kYoloModel
        << " -o " << kDetectResultPath
        << " --conf " << kYoloConf
        << " --min-height-ratio " << kMinHeightRatio
        << " --center-band " << kCenterBand
        << " 2>&1";

    const std::string output = RunCommandCapture(cmd.str());
    const bool yes = output.find("Greeting trigger candidate: YES") != std::string::npos;

    if (yes) {
        LOG("YOLO 检测结果: YES");
    } else {
        LOG("YOLO 检测结果: no");
    }

    if (output.find("Traceback") != std::string::npos ||
        output.find("error") != std::string::npos ||
        output.find("Error") != std::string::npos) {
        WARN("YOLO 输出可能包含错误: " + output.substr(0, std::min<size_t>(output.size(), 240)));
    }

    return yes;
}

static void PlayScene(int scene_id,
                      std::shared_ptr<unitree::robot::g1::AudioClient> audio,
                      std::shared_ptr<unitree::robot::g1::G1ArmActionClient> arm,
                      unitree::robot::ChannelPublisher<HandCmds>& left_pub,
                      unitree::robot::ChannelPublisher<HandCmds>& right_pub,
                      bool arm_ok) {
    if (scene_id < 0 || scene_id >= kSceneCount) {
        return;
    }

    const Scene& scene = kScenes[scene_id];
    const std::vector<uint8_t>& pcm = g_pcm_data[scene_id];
    const double pcm_duration = g_pcm_duration[scene_id];

    if (pcm.empty()) {
        WARN("场景 " + std::to_string(scene_id + 1) + " PCM 为空，跳过");
        return;
    }

    LOG("======== 自动触发场景 " + std::to_string(scene_id + 1) + " ========");
    LOG("动作: " + std::string(scene.action_name)
        + " (ExecuteAction " + std::to_string(scene.action_id) + ")");

    bool used_left_hand = false;
    bool used_right_hand = false;

    int mask = PublishHandGestures(left_pub, right_pub,
                                   scene.left_start, scene.right_start,
                                   std::chrono::milliseconds(kHandPrepareMs));
    used_left_hand = used_left_hand || ((mask & 1) != 0);
    used_right_hand = used_right_hand || ((mask & 2) != 0);

    SetVisionStatusLed(audio);

    std::ostringstream stream_id;
    stream_id << "auto_greet_" << scene_id << "_"
              << std::chrono::steady_clock::now().time_since_epoch().count();

    size_t offset = 0;
    while (offset < pcm.size() && g_running.load()) {
        const size_t n = std::min(static_cast<size_t>(kChunkSize), pcm.size() - offset);
        std::vector<uint8_t> chunk(pcm.begin() + offset, pcm.begin() + offset + n);
        audio->PlayStream("r1_auto_greet", stream_id.str(), chunk);
        offset += n;
        std::this_thread::sleep_for(100ms);
    }
    LOG("PCM 发送完毕: " + std::to_string(pcm.size() / 1024) + "KB");

    mask = PublishHandGestures(left_pub, right_pub,
                               scene.left_during, scene.right_during,
                               std::chrono::milliseconds(kHandPublishDurationMs));
    used_left_hand = used_left_hand || ((mask & 1) != 0);
    used_right_hand = used_right_hand || ((mask & 2) != 0);

    if (arm_ok && g_running.load()) {
        const int ret = arm->ExecuteAction(scene.action_id);
        if (ret != 0) {
            WARN("ExecuteAction 返回非 0: " + std::to_string(ret));
        }
    }

    const double action_duration = GetActionDuration(scene.action_id);
    const double remaining = std::max(0.0, pcm_duration - action_duration);
    const int wait_ms = static_cast<int>(remaining * 1000) +
                        static_cast<int>(kWaitBufferSec * 1000);

    int waited = 0;
    while (waited < wait_ms && g_running.load()) {
        std::this_thread::sleep_for(100ms);
        waited += 100;
    }

    const int end_hold_ms = std::max(scene.right_hold_ms, scene.left_hold_ms);
    if (end_hold_ms > 0 && g_running.load()) {
        mask = PublishHandGestures(
            left_pub, right_pub,
            scene.left_end, scene.right_end,
            std::chrono::milliseconds(std::max(kHandPublishIntervalMs, end_hold_ms)));
        used_left_hand = used_left_hand || ((mask & 1) != 0);
        used_right_hand = used_right_hand || ((mask & 2) != 0);
    }

    SetVisionStatusLed(audio);
    audio->PlayStop("r1_auto_greet");

    if (arm_ok && g_running.load()) {
        arm->ExecuteAction(kActRelease);
    }

    if (used_left_hand || used_right_hand) {
        PublishHandGestures(
            left_pub, right_pub,
            used_left_hand ? HandGesture::Open : HandGesture::None,
            used_right_hand ? HandGesture::Open : HandGesture::None,
            std::chrono::milliseconds(kHandPublishDurationMs));
    }

    LOG("场景完成，恢复人物检测");
}

int main(int argc, char** argv) {
    const std::string network = argc > 1 ? argv[1] : "eth10";

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    { std::ofstream f(kLogFile, std::ios::trunc); }

    LOG("R1 自动早安问好程序启动");
    LOG("网络接口: " + network);
    LOG("检测策略: 连续 " + std::to_string(kNeedConsecutiveYes) + " 次 YES 后随机触发场景");
    LOG("遥控器: 按 ↓+X 开始/停止视觉识别");
    LOG("动作安全边界: 只使用官方 ExecuteAction 24/25/33/19/99，不发布 rt/lowcmd");

    unitree::robot::ChannelFactory::Instance()->Init(0, network.c_str());

    auto audio = std::make_shared<unitree::robot::g1::AudioClient>();
    try {
        audio->Init();
        audio->SetTimeout(5.0f);
        audio->SetVolume(kVolume);
        LOG("语音服务就绪");
    } catch (...) {
        ERR("语音服务初始化失败");
        return 1;
    }

    auto arm = std::make_shared<unitree::robot::g1::G1ArmActionClient>();
    bool arm_ok = false;
    try {
        arm->Init();
        arm->SetTimeout(10.0f);
        std::this_thread::sleep_for(500ms);
        arm_ok = true;
        LOG("手臂动作服务就绪");
    } catch (...) {
        WARN("手臂动作服务不可用，将只播放语音和灵巧手");
    }

    unitree::robot::ChannelPublisher<HandCmds> left_pub(TOPIC_LEFT_HAND);
    left_pub.InitChannel();
    unitree::robot::ChannelPublisher<HandCmds> right_pub(TOPIC_RIGHT_HAND);
    right_pub.InitChannel();
    LOG("灵巧手 DDS Publisher 就绪");

    for (int i = 0; i < kSceneCount; ++i) {
        const std::string path = std::string(kPcmDir) + kScenes[i].pcm_file;
        if (LoadPcm(path, g_pcm_data[i], g_pcm_duration[i])) {
            LOG("加载话术 " + std::to_string(i + 1) + ": " + path
                + " (" + std::to_string(g_pcm_data[i].size() / 1024) + "KB, "
                + std::to_string(g_pcm_duration[i]) + "s)");
        }
    }

    unitree::robot::go2::VideoClient video_client;
    video_client.SetTimeout(1.0f);  // 设置较短的超时以避免卡住检测循环。
    video_client.Init();  //初始化videohub客户端
    LOG("videohub 取图客户端就绪");

    // 你应该学会: Subscriber 是“收消息的人”。这里订阅遥控器话题，只用 ↓+X 控制识别开关。
    Gamepad gamepad;
    unitree_go::msg::dds_::WirelessController_ gamepad_msg;
    std::mutex gamepad_mutex;
    auto joystick_sub = unitree::robot::ChannelSubscriber<
        unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK);
    joystick_sub.InitChannel([&](const void* msg) {
        std::lock_guard<std::mutex> lk(gamepad_mutex);
        gamepad_msg = *(unitree_go::msg::dds_::WirelessController_*)msg;
    });
    LOG("遥控器监听就绪，当前为待机状态");
    LOG("请按 ↓+X 开始视觉识别；再次按 ↓+X 停止视觉识别");

    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> scene_dist(0, kSceneCount - 1);

    int consecutive_yes = 0;

    while (g_running.load()) {
        {
            std::lock_guard<std::mutex> lk(gamepad_mutex);
            gamepad.Update(gamepad_msg);
        }

        if (gamepad.DownX()) {
            const bool next_enabled = !g_vision_enabled.load();
            g_vision_enabled.store(next_enabled);
            consecutive_yes = 0;

            if (next_enabled) {
                LOG("[↓+X] 视觉识别已开启");
            } else {
                LOG("[↓+X] 视觉识别已停止");
            }
            SetVisionStatusLed(audio);

            std::this_thread::sleep_for(300ms);
            continue;
        }

        if (!g_vision_enabled.load()) {
            std::this_thread::sleep_for(50ms);
            continue;
        }

        std::vector<uint8_t> image;
        const int32_t ret = video_client.GetImageSample(image);

        if (ret != 0 || image.empty()) {
            WARN("videohub 取图失败 ret=" + std::to_string(ret));
            consecutive_yes = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(kDetectEveryMs));
            continue;
        }

        if (!SaveJpg(image, kDetectFramePath)) {
            WARN("保存检测帧失败: " + std::string(kDetectFramePath));
            std::this_thread::sleep_for(std::chrono::milliseconds(kDetectEveryMs));
            continue;
        }

        const bool detected = DetectPersonInFrame();
        if (detected) {
            ++consecutive_yes;
        } else {
            consecutive_yes = 0;
        }

        LOG("连续检测计数: " + std::to_string(consecutive_yes) + "/"
            + std::to_string(kNeedConsecutiveYes));

        if (consecutive_yes >= kNeedConsecutiveYes && g_running.load()) {
            consecutive_yes = 0;
            const int scene_id = scene_dist(rng);
            LOG("检测到迎宾对象，随机选择场景 " + std::to_string(scene_id + 1));
            PlayScene(scene_id, audio, arm, left_pub, right_pub, arm_ok);
            LOG("场景结束，识别开关状态: " + std::string(g_vision_enabled.load() ? "开启" : "停止"));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kDetectEveryMs));
    }

    LOG("安全退出中...");
    if (arm_ok) {
        arm->ExecuteAction(kActRelease);
    }
    PublishHandGestures(left_pub, right_pub,
                        HandGesture::Open, HandGesture::Open,
                        std::chrono::milliseconds(kHandPublishDurationMs));
    audio->LedControl(0, 0, 0);
    audio->PlayStop("r1_auto_greet");
    LOG("程序已退出");
    return 0;
}
