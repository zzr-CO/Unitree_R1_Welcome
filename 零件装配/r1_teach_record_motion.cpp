/*
 * R1 dual-arm compliant teaching recorder.
 *
 * Purpose:
 *   This program records a manually taught dual-arm motion on Unitree R1.
 *   It is inspired by the G1 teaching script, but it uses R1 low-level control:
 *   - subscribe rt/lowstate
 *   - publish rt/lowcmd
 *   - keep legs and waist near the locked-standing reference pose
 *   - leave the head uncontrolled by this program, so no fixed head point is required
 *   - make both arms low-stiffness and easy to guide by hand
 *   - record left-arm and right-arm trajectories in this version
 *
 * Compile on R1 from ~/unitree_sdk2-1.0:
 *   g++ -std=c++17 test_r1_teach_record_motion.cpp \
 *       -I./include -I./thirdparty/include/ddscxx \
 *       -L./lib/aarch64 -L./thirdparty/lib/aarch64 \
 *       -lunitree_sdk2 -lddscxx -lddsc -lpthread \
 *       -Wl,-rpath,./lib/aarch64:./thirdparty/lib/aarch64 \
 *       -o test_r1_teach_record_motion
 *
 *   Note: hand DDS uses unitree_go MotorCmds_ which is part of unitree_sdk2.
 *   The same -lunitree_sdk2 library already includes the Go2 MotorCmds types.
 *
 * Run:
 *   ./test_r1_teach_record_motion eth10
 *
 * 学习目标：
 *   1. 理解“示教记录”：人手动带着机器人手臂做动作，程序把轨迹记下来。
 *   2. 理解“低刚度控制”：kp/kd 调小后，手臂更容易被人推动。
 *   3. 理解“身体保持 + 手臂示教”：腿、腰保持锁定站立，头部不检查也不保持，左右臂低刚度跟随当前角度。
 *   4. 理解“轨迹文件”：JSON 里每一帧都是一个时间 t 和一组双臂角度 q。
 *
 * 你可以改：
 *   - kDefaultNetwork：默认 DDS 网口。
 *   - kMotionName：本次记录的动作名字。
 *   - kRecordDurationSec：默认记录时长。
 *   - kRecordRateHz：JSON 记录频率。
 *   - kTeachKp / kTeachKd：手臂示教时的低刚度参数。
 *
 * 暂时不要改：
 *   - kLowCmdTopic = "rt/lowcmd"。
 *   - kLowStateTopic = "rt/lowstate"。
 *   - kR1Joints 的 IDL 编号。
 *   - kLockedStandPose 的顺序，必须和 kR1Joints 一致。
 *   - Crc32Core()，这是底层 DDS 消息的 CRC 校验。
 *
 * 安全说明：
 *   本程序会发布 rt/lowcmd，只能在 R1 调试模式下使用。
 *   不要在走跑运控模式下运行，不要和迎宾程序或其他低层控制程序同时运行。
 *   如果出现异常抖动、异响、明显阻力或姿态不合理，立刻按 Ctrl+C。
 */

// ===== 1. C++ 标准库头文件 =====
// 你应该学会：
// - array：保存固定数量的电机角度。
// - vector：保存一段时间内的多帧轨迹。
// - atomic/thread/mutex：处理 DDS 回调线程、发布线程和主线程之间的数据同步。
// - fstream：把 JSON 轨迹写入文件。
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#if !defined(_WIN32)
#include <sys/select.h>
#include <unistd.h>
#endif

// ===== 2. Unitree SDK2 头文件 =====
// LowState_：读取 R1 全身电机状态。
// LowCmd_：发送 R1 底层电机控制命令。
// ChannelSubscriber：订阅 rt/lowstate。
// ChannelPublisher：发布 rt/lowcmd。
#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/idl/go2/MotorCmds_.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

// ===== 3. 类型别名 =====
// 你应该学会：using 能把很长的类型名变短，便于阅读。
using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using LowState = unitree_hg::msg::dds_::LowState_;
using FullPose = std::array<float, 26>;
using ArmPose = std::array<float, 5>;
using DualArmPose = std::array<float, 10>;
using HandPose = std::array<float, 6>;
using DualHandPose = std::array<float, 12>;
using HandCmdsMsg = unitree_go::msg::dds_::MotorCmds_;

// ===== 4. DDS 和运行参数 =====
// 你可以改：kDefaultNetwork、kMotionName、记录时长、记录频率、示教 kp/kd。
// 暂时不要改：kLowCmdTopic、kLowStateTopic、kControlPeriodUs。
static constexpr const char* kLowCmdTopic = "rt/lowcmd";
static constexpr const char* kLowStateTopic = "rt/lowstate";
static constexpr const char* kRightHandTopic = "rt/brainco/right/cmd";
static constexpr const char* kLeftHandTopic = "rt/brainco/left/cmd";
static constexpr const char* kDefaultNetwork = "eth10";
static constexpr const char* kMotionName = "r1_dual_arm_assembly_teach";
static constexpr const char* kOutputFilename = "R1-hand.json";
static constexpr int kControlPeriodUs = 2000;      // 500Hz 底层命令发布频率
static constexpr double kRecordDurationSec = 60.0; // 默认记录 60 秒，避免示教中途太快退出低刚度
static constexpr double kRecordRateHz = 30.0;      // 默认每秒记录 30 帧
static constexpr float kTeachKp = 1.0f;            // 手臂低刚度，越小越软
static constexpr float kTeachKd = 0.10f;           // 手臂阻尼，调小后更容易拖动；太小会显得松
static constexpr float kStandKp = 90.0f;           // 腿、腰保持站立的刚度；站不稳时先加 Kp
static constexpr float kStandKd = 5.0f;            // 腿、腰保持站立的阻尼；Kp 变大后适当加 Kd 防抖
static constexpr float kMaxBodyStartErrorRad = 0.35f;
static constexpr float kZero = 0.0f;
static constexpr float kHandSpeed = 1.0f;        // 手部运动速度
static constexpr int kHandPublishPeriodMs = 100;  // 手部发布间隔

// ===== 5. 关节信息结构体 =====
// idl_index：LowState/LowCmd 里的真实电机编号。
// pose_index：在 26 个真实电机数组里的位置。
// is_teach_arm：左右手臂关节都会使用低刚度跟随当前实际角度。
struct JointInfo {
    int idl_index;
    const char* english_name;
    const char* chinese_name;
    const char* group_name;
    bool is_teach_arm;
};

// ===== 6. R1 26 个真实电机表 =====
// 文档里的 14、20、21、27、28 是 EMPTY，所以这里跳过。
// 暂时不要改 IDL 编号；如果只是显示不顺眼，可以改中文名。
static const std::array<JointInfo, 26> kR1Joints{{
    {0,  "L_LEG_HIP_PITCH",   "左髋前后",   "左腿", false},
    {1,  "L_LEG_HIP_ROLL",    "左髋左右",   "左腿", false},
    {2,  "L_LEG_HIP_YAW",     "左髋旋转",   "左腿", false},
    {3,  "L_LEG_KNEE",        "左膝",       "左腿", false},
    {4,  "L_LEG_ANKLE_PITCH", "左踝前后",   "左腿", false},
    {5,  "L_LEG_ANKLE_ROLL",  "左踝左右",   "左腿", false},
    {6,  "R_LEG_HIP_PITCH",   "右髋前后",   "右腿", false},
    {7,  "R_LEG_HIP_ROLL",    "右髋左右",   "右腿", false},
    {8,  "R_LEG_HIP_YAW",     "右髋旋转",   "右腿", false},
    {9,  "R_LEG_KNEE",        "右膝",       "右腿", false},
    {10, "R_LEG_ANKLE_PITCH", "右踝前后",   "右腿", false},
    {11, "R_LEG_ANKLE_ROLL",  "右踝左右",   "右腿", false},
    {12, "WAIST_ROLL",        "腰左右倾斜", "腰部", false},
    {13, "WAIST_YAW",         "腰部旋转",   "腰部", false},
    {15, "L_SHOULDER_PITCH",  "左肩前后",   "左臂", true},
    {16, "L_SHOULDER_ROLL",   "左肩左右",   "左臂", true},
    {17, "L_SHOULDER_YAW",    "左肩旋转",   "左臂", true},
    {18, "L_ELBOW",           "左肘",       "左臂", true},
    {19, "L_WRIST_ROLL",      "左腕旋转",   "左臂", true},
    {22, "R_SHOULDER_PITCH",  "右肩前后",   "右臂", true},
    {23, "R_SHOULDER_ROLL",   "右肩左右",   "右臂", true},
    {24, "R_SHOULDER_YAW",    "右肩旋转",   "右臂", true},
    {25, "R_ELBOW",           "右肘",       "右臂", true},
    {26, "R_WRIST_ROLL",      "右腕旋转",   "右臂", true},
    {29, "HEAD_PITCH",        "头部俯仰",   "头部", false},
    {30, "HEAD_YAW",          "头部左右转", "头部", false},
}};

// ===== 7. 锁定站立基准姿态 =====
// 这个数组来自你刚刚记录的 100 帧平均值。
// 顺序必须和 kR1Joints 完全一致，不是 0-30 连续编号。
static const FullPose kLockedStandPose = {
    -0.285902f, 0.049563f, -0.148795f, 0.425323f,
    -0.184718f, -0.055339f, -0.324874f, -0.043069f,
    0.139767f, 0.419463f, -0.151288f, 0.067168f,
    -0.003439f, -0.000778f, 0.199532f, 0.180349f,
    -0.000526f, 0.810018f, -0.000011f, 0.201036f,
    -0.186743f, 0.003484f, 0.809403f, -0.001595f,
    0.003146f, 0.001092f
};

// ===== 8. 左右臂在 26 个真实电机数组里的下标 =====
// 左臂对应 kR1Joints 的第 14-18 项，右臂对应第 19-23 项。
// 你应该学会：数组下标和 IDL 电机编号不是同一个东西。
static const std::array<size_t, 5> kLeftArmPoseIndices{{14, 15, 16, 17, 18}};
static const std::array<size_t, 5> kRightArmPoseIndices{{19, 20, 21, 22, 23}};

// ===== 8b. 灵巧手常用手势预设 =====
// 手指顺序: [拇指, 拇指副指, 食指, 中指, 无名指, 小指]
// 范围: 0.0=张开, 1.0=全握
// 你可以改：每个手势里的左右手 6 个 float 数值。
static const HandPose kHandOpen       {{0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f}};
static const HandPose kHandClose      {{1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f}};
static const HandPose kHandHalfFist   {{0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f}};
static const HandPose kLeftTaskGrip   {{0.00f, 0.00f, 0.20f, 0.20f, 0.20f, 0.20f}};
static const HandPose kRightTaskGrip  {{0.60f, 0.60f, 0.70f, 0.70f, 0.70f, 0.70f}};
static const HandPose kHandPinch      {{0.65f, 0.65f, 0.65f, 0.20f, 0.20f, 0.20f}};
static const HandPose kHandVSign      {{1.00f, 1.00f, 0.00f, 0.00f, 1.00f, 1.00f}};

struct NamedHandPose {
    const char* key;
    const char* name;
    HandPose left_pose;
    HandPose right_pose;
};

static const std::array<NamedHandPose, 6> kHandPresets{{
    {"0", "open / 张开",                         kHandOpen,     kHandOpen},
    {"1", "close / 全握",                        kHandClose,    kHandClose},
    {"2", "half-fist / 半握拳",                  kHandHalfFist, kHandHalfFist},
    {"3", "task-grip / 实测任务动作",            kLeftTaskGrip, kRightTaskGrip},
    {"4", "pinch / 捏取",                        kHandPinch,    kHandPinch},
    {"5", "v-sign / 剪刀手",                     kHandVSign,    kHandVSign},
}};

// ===== 9. 一帧记录数据 =====
// t：从记录开始算起的时间，单位秒。
// q：双臂 10 个关节角，单位 rad。
// h：双手 12 个手指位置，0.0=张开, 1.0=全握。
// 顺序固定为：左臂 5 个 + 右臂 5 个, 左手 6 指 + 右手 6 指。
struct RecordedFrame {
    double t = 0.0;
    DualArmPose q{};
    DualHandPose h{};
};

// ===== 10. 跨线程共享状态 =====
// DDS 回调线程会持续更新 g_latest_full_pose。
// 发布线程和主线程读取它时必须加锁。
static std::atomic<bool> g_running{true};
static std::atomic<bool> g_control_enabled{false};
static std::mutex g_state_mutex;
static FullPose g_latest_full_pose{};
static uint8_t g_mode_machine = 1;
static uint64_t g_state_seq = 0;
static bool g_state_valid = false;

// ===== 10b. 灵巧手共享状态 =====
// left_hand_pose / right_hand_pose：当前要发布给灵巧手的目标位置。
// 主线程通过 '0'-'5' 按键修改，录制时逐帧记录到 JSON。
static std::mutex g_hand_mutex;
static HandPose g_left_hand_pose = kHandOpen;
static HandPose g_right_hand_pose = kHandOpen;

// ===== 11. Ctrl+C 信号处理 =====
// 只做最小工作：关掉运行开关和控制开关。
// 真正的 disable 帧在主流程里发送。
static void SignalHandler(int) {
    g_control_enabled.store(false);
    g_running.store(false);
}

// ===== 12. CRC 校验函数 =====
// 暂时不要改：这是 Unitree 底层示例常用写法。
static uint32_t Crc32Core(uint32_t* ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t crc32 = 0xFFFFFFFF;
    const uint32_t polynomial = 0x04c11db7;

    for (uint32_t i = 0; i < len; i++) {
        xbit = 1U << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (crc32 & 0x80000000) {
                crc32 <<= 1;
                crc32 ^= polynomial;
            } else {
                crc32 <<= 1;
            }
            if (data & xbit) {
                crc32 ^= polynomial;
            }
            xbit >>= 1;
        }
    }
    return crc32;
}

// ===== 13. 弧度转角度 =====
// 控制代码用 rad，打印给人看时用 deg 更直观。
static float RadToDeg(float rad) {
    return rad * 180.0f / 3.14159265358979323846f;
}

static bool IsHeadJoint(const JointInfo& joint) {
    return std::strcmp(joint.group_name, "头部") == 0;
}

// ===== 13b. 灵巧手辅助函数 =====
static void SetHandPose(HandCmdsMsg& msg, const HandPose& pose) {
    msg.cmds().resize(6);
    for (size_t i = 0; i < pose.size(); ++i) {
        msg.cmds()[i].q() = pose[i];
        msg.cmds()[i].dq() = kHandSpeed;
    }
}

static void SetCurrentHandPoses(const HandPose& left, const HandPose& right) {
    std::lock_guard<std::mutex> lock(g_hand_mutex);
    g_left_hand_pose = left;
    g_right_hand_pose = right;
}

static void GetCurrentHandPoses(HandPose& left, HandPose& right) {
    std::lock_guard<std::mutex> lock(g_hand_mutex);
    left = g_left_hand_pose;
    right = g_right_hand_pose;
}

static DualHandPose GetCurrentDualHandPose() {
    std::lock_guard<std::mutex> lock(g_hand_mutex);
    DualHandPose dual{};
    for (size_t i = 0; i < 6; ++i) {
        dual.at(i) = g_left_hand_pose.at(i);
        dual.at(i + 6) = g_right_hand_pose.at(i);
    }
    return dual;
}

// 灵巧手发布线程：以 100ms 间隔持续向左右手发送当前目标位置
static void HandWriterLoop(
    unitree::robot::ChannelPublisher<HandCmdsMsg>* left_pub,
    unitree::robot::ChannelPublisher<HandCmdsMsg>* right_pub) {
    while (g_running.load()) {
        HandPose left_pose{}, right_pose{};
        GetCurrentHandPoses(left_pose, right_pose);

        HandCmdsMsg left_msg, right_msg;
        SetHandPose(left_msg, left_pose);
        SetHandPose(right_msg, right_pose);

        left_pub->Write(left_msg);
        right_pub->Write(right_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(kHandPublishPeriodMs));
    }
}

static void PrintDualHandPose(const DualHandPose& dual, const char* title) {
    std::cout << "\n" << title << "\n";
    std::cout << "左手: [";
    for (size_t i = 0; i < 6; ++i) {
        std::cout << std::fixed << std::setprecision(2) << dual.at(i);
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]\n右手: [";
    for (size_t i = 6; i < 12; ++i) {
        std::cout << std::fixed << std::setprecision(2) << dual.at(i);
        if (i < 11) std::cout << ", ";
    }
    std::cout << "]\n" << std::defaultfloat;
}

// ===== 13c. 解析并应用灵巧手手势输入 =====
// 支持：
// - 0-5：双手一起变成某个手势；其中 3 会使用左右手各自的实测任务动作。
// - l0-l5：只改左手；其中 l3 是左手实测任务动作。
// - r0-r5：只改右手；其中 r3 是右手实测任务动作。
// 你应该学会：把重复逻辑封装成函数，主菜单和录制过程就不会各写一份。
static bool ApplyHandCommand(const std::string& input, bool verbose) {
    if (input.empty()) return false;

    if ((input.at(0) == 'l' || input.at(0) == 'L') && input.size() >= 2) {
        const char digit = input.at(1);
        for (const auto& hp : kHandPresets) {
            if (digit == hp.key[0]) {
                HandPose left, right;
                GetCurrentHandPoses(left, right);
                SetCurrentHandPoses(hp.left_pose, right);
                if (verbose) {
                    std::cout << "左手已设置为: " << hp.name << "（右手不变）\n";
                }
                return true;
            }
        }
        return false;
    }

    if ((input.at(0) == 'r' || input.at(0) == 'R') && input.size() >= 2) {
        const char digit = input.at(1);
        for (const auto& hp : kHandPresets) {
            if (digit == hp.key[0]) {
                HandPose left, right;
                GetCurrentHandPoses(left, right);
                SetCurrentHandPoses(left, hp.right_pose);
                if (verbose) {
                    std::cout << "右手已设置为: " << hp.name << "（左手不变）\n";
                }
                return true;
            }
        }
        return false;
    }

    for (const auto& hp : kHandPresets) {
        if (input == hp.key || input.at(0) == hp.key[0]) {
            SetCurrentHandPoses(hp.left_pose, hp.right_pose);
            if (verbose) {
                std::cout << "双手已设置为: " << hp.name << "\n";
            }
            return true;
        }
    }

    return false;
}

// ===== 13d. 录制期间非阻塞读取键盘输入 =====
// 正常 getline 会卡住程序，不适合录制中使用。
// 这里在 Linux/R1 上用 select() 检查键盘有没有输入：没有输入就立刻返回，不影响录制。
// 你可以改：不用改。录制时输入 0-5、l0-l5、r0-r5 后回车即可改变并记录手势。
static bool TryReadLineNonBlocking(std::string& line) {
#if defined(_WIN32)
    (void)line;
    return false;
#else
    static std::string buffer;
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(STDIN_FILENO, &read_set);

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    const int ready = select(STDIN_FILENO + 1, &read_set, nullptr, nullptr, &timeout);
    if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &read_set)) {
        return false;
    }

    char ch = '\0';
    while (read(STDIN_FILENO, &ch, 1) == 1) {
        if (ch == '\n' || ch == '\r') {
            line = buffer;
            buffer.clear();
            return true;
        }
        buffer.push_back(ch);

        FD_ZERO(&read_set);
        FD_SET(STDIN_FILENO, &read_set);
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
        if (select(STDIN_FILENO + 1, &read_set, nullptr, nullptr, &timeout) <= 0) {
            break;
        }
    }

    return false;
#endif
}

// ===== 14. LowState 回调函数 =====
// 每收到一帧 rt/lowstate，SDK 自动调用这里。
// 这个函数只读取状态，不发布控制。
static void LowStateHandler(const void* message) {
    LowState state = *(const LowState*)message;

    const uint32_t expected_crc =
        Crc32Core((uint32_t*)&state, (sizeof(LowState) >> 2) - 1);
    if (state.crc() != expected_crc) {
        std::cerr << "[WARN] lowstate CRC mismatch, skip this frame\n";
        return;
    }

    FullPose pose{};
    for (size_t i = 0; i < kR1Joints.size(); ++i) {
        pose.at(i) = state.motor_state().at(kR1Joints.at(i).idl_index).q();
    }

    std::lock_guard<std::mutex> lock(g_state_mutex);
    g_latest_full_pose = pose;
    g_mode_machine = state.mode_machine();
    ++g_state_seq;
    g_state_valid = true;
}

// ===== 15. 读取最新全身姿态 =====
// 返回一份拷贝，避免主线程直接读取 DDS 回调正在修改的数据。
static bool GetLatestFullPose(FullPose& pose, uint8_t& mode_machine, uint64_t& seq) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (!g_state_valid) return false;
    pose = g_latest_full_pose;
    mode_machine = g_mode_machine;
    seq = g_state_seq;
    return true;
}

// ===== 16. 从全身姿态中取一条手臂 5 个关节 =====
// 你应该学会：同一个函数可以通过传入不同下标表，复用给左臂和右臂。
static ArmPose ExtractArm(const FullPose& pose, const std::array<size_t, 5>& indices) {
    ArmPose arm{};
    for (size_t i = 0; i < indices.size(); ++i) {
        arm.at(i) = pose.at(indices.at(i));
    }
    return arm;
}

// ===== 17. 从全身姿态中取双臂 10 个关节 =====
// JSON 的 q 就按这个顺序保存：先左臂，后右臂。
static DualArmPose ExtractDualArm(const FullPose& pose) {
    DualArmPose dual{};
    for (size_t i = 0; i < kLeftArmPoseIndices.size(); ++i) {
        dual.at(i) = pose.at(kLeftArmPoseIndices.at(i));
    }
    for (size_t i = 0; i < kRightArmPoseIndices.size(); ++i) {
        dual.at(i + kLeftArmPoseIndices.size()) = pose.at(kRightArmPoseIndices.at(i));
    }
    return dual;
}

// ===== 18. 生成时间戳字符串 =====
// 用于默认 JSON 文件名，避免每次覆盖旧记录。
static std::string MakeTimestamp() {
    const std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &now);
#else
    localtime_r(&now, &tm_buf);
#endif
    char buffer[32] = {};
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &tm_buf);
    return std::string(buffer);
}

// ===== 19. 生成默认输出文件名 =====
// 你可以改：kOutputFilename。
// 说明：
// - 现在固定输出 R1-hand.json，方便后续回放程序直接读取固定文件。
// - 每次录制会覆盖上一次 R1-hand.json；如果你想保留历史文件，需要手动改名备份。
static std::string DefaultOutputPath() {
    return kOutputFilename;
}

// ===== 20. JSON 字符串转义 =====
// 因为我们不引入第三方 JSON 库，所以要手写最基本的字符串转义。
static std::string JsonEscape(const std::string& text) {
    std::ostringstream out;
    for (char ch : text) {
        if (ch == '\\') {
            out << "\\\\";
        } else if (ch == '"') {
            out << "\\\"";
        } else if (ch == '\n') {
            out << "\\n";
        } else {
            out << ch;
        }
    }
    return out.str();
}

// ===== 20b. JSON 数组写入工具 =====
// 你应该学会：轨迹文件最怕“顺序不清楚”。这里统一输出数组，后续回放代码按同一顺序读取。
template <typename ArrayLike>
static void WriteNumberArray(std::ostream& out, const ArrayLike& values) {
    out << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) out << ", ";
        out << values.at(i);
    }
    out << "]";
}

static void WriteArmJointIdArray(std::ostream& out) {
    out << "[";
    for (size_t i = 0; i < kLeftArmPoseIndices.size(); ++i) {
        if (i > 0) out << ", ";
        out << kR1Joints.at(kLeftArmPoseIndices.at(i)).idl_index;
    }
    for (size_t i = 0; i < kRightArmPoseIndices.size(); ++i) {
        out << ", " << kR1Joints.at(kRightArmPoseIndices.at(i)).idl_index;
    }
    out << "]";
}

static void WriteStringList(std::ostream& out, const std::vector<std::string>& values) {
    out << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) out << ", ";
        out << "\"" << JsonEscape(values.at(i)) << "\"";
    }
    out << "]";
}

// ===== 21. 打印单条手臂角度 =====
// 你可以改 setprecision(3)，比如改成 4 会显示更多小数。
static void PrintArmPose(const ArmPose& pose,
                         const std::array<size_t, 5>& indices,
                         const std::string& title,
                         uint64_t seq = 0) {
    std::cout << "\n" << title;
    if (seq > 0) std::cout << " seq=" << seq;
    std::cout << "\n";
    std::cout << "----------------------------------------------------------------\n";
    std::cout << std::left
              << std::setw(5) << "IDL"
              << std::setw(14) << "关节"
              << std::setw(20) << "英文代号"
              << std::right
              << std::setw(10) << "q(rad)"
              << std::setw(10) << "q(deg)"
              << "\n";

    std::cout << std::fixed << std::setprecision(3);
    for (size_t i = 0; i < indices.size(); ++i) {
        const JointInfo& joint = kR1Joints.at(indices.at(i));
        std::cout << std::left
                  << std::setw(5) << joint.idl_index
                  << std::setw(14) << joint.chinese_name
                  << std::setw(20) << joint.english_name
                  << std::right
                  << std::setw(10) << pose.at(i)
                  << std::setw(10) << RadToDeg(pose.at(i))
                  << "\n";
    }
    std::cout << std::defaultfloat;
}

// ===== 22. 打印当前双臂角度 =====
// 你应该学会：显示和保存可以共用同一组关节下标，避免顺序混乱。
static void PrintDualArmPose(const FullPose& pose, const std::string& title, uint64_t seq = 0) {
    std::cout << "\n" << title;
    if (seq > 0) std::cout << " seq=" << seq;
    std::cout << "\n";
    PrintArmPose(ExtractArm(pose, kLeftArmPoseIndices), kLeftArmPoseIndices, "Left arm / 左臂");
    PrintArmPose(ExtractArm(pose, kRightArmPoseIndices), kRightArmPoseIndices, "Right arm / 右臂");
}

// ===== 23. 打印 JSON 帧里的双臂角度 =====
// RecordedFrame 里已经没有全身姿态，只有双臂 10 个 q，所以需要拆成左右臂再打印。
static void PrintRecordedDualArmPose(const DualArmPose& pose, const std::string& title) {
    FullPose full{};
    for (size_t i = 0; i < kLeftArmPoseIndices.size(); ++i) {
        full.at(kLeftArmPoseIndices.at(i)) = pose.at(i);
    }
    for (size_t i = 0; i < kRightArmPoseIndices.size(); ++i) {
        full.at(kRightArmPoseIndices.at(i)) = pose.at(i + kLeftArmPoseIndices.size());
    }
    PrintDualArmPose(full, title);
}

// ===== 24. 检查当前身体姿态是否接近锁定站立基准 =====
// 安全意义：
// - 如果机器人腿、腰不是锁定站立，直接命令它去 kLockedStandPose 可能产生明显动作。
// - 左右臂都允许后续自由示教，所以安全检查不再用手臂位置拦截开始。
// - 头部不参与检查，也不由本程序保持点位。
static bool CheckBodyCloseToLockedStand(const FullPose& current) {
    float max_error = 0.0f;
    const JointInfo* max_joint = nullptr;

    for (size_t i = 0; i < kR1Joints.size(); ++i) {
        const JointInfo& joint = kR1Joints.at(i);
        if (joint.is_teach_arm) continue;
        if (IsHeadJoint(joint)) continue;

        const float error = std::fabs(current.at(i) - kLockedStandPose.at(i));
        if (error > max_error) {
            max_error = error;
            max_joint = &joint;
        }
    }

    if (max_error <= kMaxBodyStartErrorRad) {
        return true;
    }

    std::cout << "\n[SAFETY] 当前身体姿态和锁定站立基准差异过大，拒绝开始示教。\n";
    if (max_joint != nullptr) {
        std::cout << "最大差异关节: " << max_joint->english_name
                  << " / " << max_joint->chinese_name
                  << ", error=" << max_error
                  << " rad (" << RadToDeg(max_error) << " deg)\n";
    }
    std::cout << "请先确认机器人腿和腰处于调试模式的锁定站立姿态，再重新输入 r。\n";
    return false;
}

// ===== 25. 清空 LowCmd =====
// 默认把所有电机禁用，后面再按需要启用 26 个真实电机。
static void DisableAllMotors(LowCmd& cmd) {
    for (size_t i = 0; i < cmd.motor_cmd().size(); ++i) {
        cmd.motor_cmd().at(i).mode() = 0;
        cmd.motor_cmd().at(i).tau() = kZero;
        cmd.motor_cmd().at(i).q() = kZero;
        cmd.motor_cmd().at(i).dq() = kZero;
        cmd.motor_cmd().at(i).kp() = kZero;
        cmd.motor_cmd().at(i).kd() = kZero;
    }
}

// ===== 26. 根据身体部位选择保持参数 =====
// 腿、腰需要更强保持；左右臂都低刚度；头部不由本程序控制。
static void SelectHoldGains(const JointInfo& joint, float& kp, float& kd) {
    if (joint.is_teach_arm) {
        kp = kTeachKp;
        kd = kTeachKd;
    } else if (std::strcmp(joint.group_name, "左腿") == 0 ||
               std::strcmp(joint.group_name, "右腿") == 0 ||
               std::strcmp(joint.group_name, "腰部") == 0) {
        kp = kStandKp;
        kd = kStandKd;
    } else {
        kp = kZero;
        kd = kZero;
    }
}

// ===== 27. 填充全身命令 =====
// 核心逻辑：
// - 腿、腰：命令到 kLockedStandPose。
// - 头部：跳过，不发布保持命令。
// - 左右臂：命令到当前实际角度，且 kp/kd 很低，方便人手动带动。
static void FillFullBodyTeachCommand(LowCmd& cmd, const FullPose& latest_pose) {
    for (size_t i = 0; i < kR1Joints.size(); ++i) {
        const JointInfo& joint = kR1Joints.at(i);
        if (IsHeadJoint(joint)) {
            continue;
        }

        const int id = joint.idl_index;
        float kp = 0.0f;
        float kd = 0.0f;
        SelectHoldGains(joint, kp, kd);

        const float target_q = joint.is_teach_arm ? latest_pose.at(i) : kLockedStandPose.at(i);
        cmd.motor_cmd().at(id).mode() = 1;
        cmd.motor_cmd().at(id).tau() = kZero;
        cmd.motor_cmd().at(id).q() = target_q;
        cmd.motor_cmd().at(id).dq() = kZero;
        cmd.motor_cmd().at(id).kp() = kp;
        cmd.motor_cmd().at(id).kd() = kd;
    }
}

// ===== 28. 发送 disable 帧 =====
// 退出时连续发几帧禁用命令，避免程序停止后仍占用底层控制。
static void PublishDisableFrames(unitree::robot::ChannelPublisher<LowCmd>& publisher,
                                 int frame_count) {
    FullPose latest_pose{};
    uint8_t mode_machine = 1;
    uint64_t seq = 0;
    GetLatestFullPose(latest_pose, mode_machine, seq);

    LowCmd cmd;
    DisableAllMotors(cmd);
    cmd.mode_pr() = 0;
    cmd.mode_machine() = mode_machine;

    for (int i = 0; i < frame_count; ++i) {
        cmd.crc() = Crc32Core((uint32_t*)&cmd, (sizeof(cmd) >> 2) - 1);
        publisher.Write(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

// ===== 29. 底层命令发布线程 =====
// g_control_enabled=true 时，它会以 500Hz 持续发送 rt/lowcmd。
// 你应该学会：底层控制不能只发一帧，要周期性持续发布。
static void CommandWriterLoop(unitree::robot::ChannelPublisher<LowCmd>* publisher) {
    while (g_running.load()) {
        if (!g_control_enabled.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        FullPose latest_pose{};
        uint8_t mode_machine = 1;
        uint64_t seq = 0;
        if (!GetLatestFullPose(latest_pose, mode_machine, seq)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        LowCmd cmd;
        DisableAllMotors(cmd);
        cmd.mode_pr() = 0;
        cmd.mode_machine() = mode_machine;
        FillFullBodyTeachCommand(cmd, latest_pose);
        cmd.crc() = Crc32Core((uint32_t*)&cmd, (sizeof(cmd) >> 2) - 1);
        publisher->Write(cmd);

        std::this_thread::sleep_for(std::chrono::microseconds(kControlPeriodUs));
    }
}

// ===== 30. 倒计时 =====
// 开始记录前给你时间把手放到合适位置、准备带动双臂。
static void Countdown() {
    std::cout << "Press Ctrl+C anytime if the robot behaves abnormally.\n";
    for (int value = 3; value >= 1; --value) {
        std::cout << value << "...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Recording. Move the arms by hand now.\n";
}

// ===== 31. 写 JSON 轨迹文件 =====
// 这里手写 JSON，避免 R1 上额外安装第三方 JSON 库。
static bool WriteMotionJson(const std::string& output_path,
                            const std::vector<RecordedFrame>& frames,
                            double duration_sec) {
    if (frames.empty()) {
        std::cout << "[WARN] 没有记录到轨迹帧，不写 JSON 文件。\n";
        return false;
    }

    std::ofstream out(output_path);
    if (!out) {
        std::cerr << "[ERROR] 无法写入文件: " << output_path << "\n";
        return false;
    }

    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "  \"format\": \"r1_dual_arm_hand_motion_v1\",\n";
    out << "  \"schema_version\": 1,\n";
    out << "  \"name\": \"" << JsonEscape(kMotionName) << "\",\n";
    out << "  \"created_at\": \"" << MakeTimestamp() << "\",\n";
    out << "  \"recording_mode\": \"compliant_teach_lowcmd\",\n";
    out << "  \"units\": \"radians_for_arm__normalized_0to1_for_hand\",\n";
    out << "  \"replay_note\": \"frames[].q is dual-arm command trajectory; frames[].h is dual-hand command trajectory\",\n";
    out << "  \"duration\": " << duration_sec << ",\n";
    out << "  \"sample_rate_hz\": " << kRecordRateHz << ",\n";
    out << "  \"teach_kp\": " << kTeachKp << ",\n";
    out << "  \"teach_kd\": " << kTeachKd << ",\n";
    out << "  \"lowcmd_topic\": \"" << kLowCmdTopic << "\",\n";
    out << "  \"lowstate_topic\": \"" << kLowStateTopic << "\",\n";
    out << "  \"left_hand_cmd_topic\": \"" << kLeftHandTopic << "\",\n";
    out << "  \"right_hand_cmd_topic\": \"" << kRightHandTopic << "\",\n";
    out << "  \"locked_stand_pose_order\": \"kR1Joints\",\n";
    out << "  \"locked_stand_pose\": ";
    WriteNumberArray(out, kLockedStandPose);
    out << ",\n";
    out << "  \"arm_joint_count\": 10,\n";
    out << "  \"hand_finger_count\": 12,\n";
    out << "  \"arm_order\": \"left_arm_5_then_right_arm_5\",\n";
    out << "  \"hand_order\": \"left_hand_6_then_right_hand_6\",\n";
    out << "  \"finger_order\": \"[thumb,thumb_aux,index,middle,ring,pinky]\",\n";
    out << "  \"arm_joint_idl_indices\": ";
    WriteArmJointIdArray(out);
    out << ",\n";
    out << "  \"joint_names\": ";
    WriteStringList(out, {
        "L_SHOULDER_PITCH", "L_SHOULDER_ROLL", "L_SHOULDER_YAW", "L_ELBOW", "L_WRIST_ROLL",
        "R_SHOULDER_PITCH", "R_SHOULDER_ROLL", "R_SHOULDER_YAW", "R_ELBOW", "R_WRIST_ROLL"
    });
    out << ",\n";
    out << "  \"joint_names_cn\": ";
    WriteStringList(out, {
        "左肩前后", "左肩左右", "左肩旋转", "左肘", "左腕旋转",
        "右肩前后", "右肩左右", "右肩旋转", "右肘", "右腕旋转"
    });
    out << ",\n";
    out << "  \"hand_finger_names\": ";
    WriteStringList(out, {
        "L_THUMB", "L_THUMB_AUX", "L_INDEX", "L_MIDDLE", "L_RING", "L_PINKY",
        "R_THUMB", "R_THUMB_AUX", "R_INDEX", "R_MIDDLE", "R_RING", "R_PINKY"
    });
    out << ",\n";
    out << "  \"hand_finger_names_cn\": ";
    WriteStringList(out, {
        "左拇指", "左拇指副指", "左食指", "左中指", "左无名指", "左小指",
        "右拇指", "右拇指副指", "右食指", "右中指", "右无名指", "右小指"
    });
    out << ",\n";
    out << "  \"frames\": [\n";
    for (size_t frame_i = 0; frame_i < frames.size(); ++frame_i) {
        const RecordedFrame& frame = frames.at(frame_i);
        out << "    {\"t\": " << frame.t << ", \"q\": ";
        WriteNumberArray(out, frame.q);
        out << ", \"h\": ";
        WriteNumberArray(out, frame.h);
        out << "}";
        if (frame_i + 1 < frames.size()) out << ",";
        out << "\n";
    }
    out << "  ]\n";
    out << "}\n";
    return true;
}

// ===== 32. 执行一次示教记录 =====
// 流程：
// 1. 确认当前身体姿态接近锁定站立。
// 2. 倒计时。
// 3. 打开 rt/lowcmd 发布。
// 4. 记录双臂实际角度。
// 5. 关闭控制并保存 JSON。
static void RecordMotion(unitree::robot::ChannelPublisher<LowCmd>& publisher) {
    FullPose start_pose{};
    uint8_t mode_machine = 1;
    uint64_t seq = 0;
    if (!GetLatestFullPose(start_pose, mode_machine, seq)) {
        std::cout << "[WARN] 还没有收到 rt/lowstate，无法开始记录。\n";
        return;
    }

    if (!CheckBodyCloseToLockedStand(start_pose)) {
        return;
    }

    PrintDualArmPose(start_pose, "记录开始前双臂角度", seq);
    PrintDualHandPose(GetCurrentDualHandPose(), "当前灵巧手姿态");
    std::cout << "Record duration: " << kRecordDurationSec << " sec\n";
    std::cout << "Record rate: " << kRecordRateHz << " Hz\n";
    std::cout << "Teach gains: kp=" << kTeachKp << ", kd=" << kTeachKd << "\n";
    std::cout << "During recording, you can type hand commands then press Enter:\n";
    std::cout << "  0-5 = both hands, l0-l5 = left hand, r0-r5 = right hand\n";
    std::cout << "  3/l3/r3 = 实测任务动作：左手[0,0,0.2,0.2,0.2,0.2]，右手[0.6,0.6,0.7,0.7,0.7,0.7]\n";
    std::cout << "  s = stop and save / 提前结束并保存\n";
    std::cout << "Output JSON will overwrite: " << kOutputFilename << "\n";
    Countdown();
    if (!g_running.load()) {
        std::cout << "Recording canceled before control started.\n";
        return;
    }

    std::vector<RecordedFrame> frames;
    frames.reserve(static_cast<size_t>(kRecordDurationSec * kRecordRateHz) + 8);

    g_control_enabled.store(true);
    const auto start_time = std::chrono::steady_clock::now();
    const auto end_time = start_time + std::chrono::milliseconds(
        static_cast<int>(kRecordDurationSec * 1000.0));
    auto next_tick = start_time;
    const auto record_period = std::chrono::microseconds(
        static_cast<int>(1000000.0 / kRecordRateHz));
    bool stop_and_save_requested = false;

    while (g_running.load() &&
           !stop_and_save_requested &&
           std::chrono::steady_clock::now() < end_time) {
        const auto now = std::chrono::steady_clock::now();
        if (now < next_tick) {
            auto sleep_duration = next_tick - now;
            if (sleep_duration > std::chrono::milliseconds(2)) {
                sleep_duration = std::chrono::milliseconds(2);
            }
            std::this_thread::sleep_for(sleep_duration);
            continue;
        }

        std::string hand_input;
        if (TryReadLineNonBlocking(hand_input)) {
            if (hand_input == "s" || hand_input == "S") {
                std::cout << "收到 s：提前结束录制并保存。\n";
                stop_and_save_requested = true;
            } else if (ApplyHandCommand(hand_input, true)) {
                PrintDualHandPose(GetCurrentDualHandPose(), "录制中灵巧手姿态已更新");
            } else if (!hand_input.empty()) {
                std::cout << "[WARN] 录制中忽略无效手势输入: " << hand_input << "\n";
            }
        }

        if (stop_and_save_requested) {
            break;
        }

        FullPose latest_pose{};
        if (GetLatestFullPose(latest_pose, mode_machine, seq)) {
            RecordedFrame frame;
            frame.t = std::chrono::duration<double>(now - start_time).count();
            frame.q = ExtractDualArm(latest_pose);
            frame.h = GetCurrentDualHandPose();
            frames.push_back(frame);
        }
        next_tick += record_period;
    }

    g_control_enabled.store(false);
    PublishDisableFrames(publisher, 20);

    const double actual_duration =
        frames.empty() ? 0.0 : frames.back().t;
    const std::string output_path = DefaultOutputPath();
    if (WriteMotionJson(output_path, frames, actual_duration)) {
        std::cout << "Saved " << frames.size() << " frames to " << output_path << "\n";
        PrintRecordedDualArmPose(frames.front().q, "first_frame");
        PrintRecordedDualArmPose(frames.back().q, "last_frame");
        PrintDualHandPose(frames.front().h, "first_hand_pose");
        PrintDualHandPose(frames.back().h, "last_hand_pose");
    }
}

// ===== 33. 打印操作菜单 =====
// 你应该学会：危险测试程序要用清晰菜单减少误操作。
static void PrintMenu() {
    std::cout << "\n================ R1 双臂+灵巧手低刚度示教记录 ================\n";
    std::cout << "  v = 查看当前双臂角度\n";
    std::cout << "  hh = 查看当前灵巧手姿态\n";
    std::cout << "  r = 倒计时后开始记录双臂示教轨迹\n";
    std::cout << "  h = 显示菜单\n";
    std::cout << "  q = 退出\n";
    std::cout << "  Ctrl+C = 停止控制并退出\n";
    std::cout << "  录制中输入 s 回车 = 提前结束并保存 JSON\n";
    std::cout << "\n灵巧手控制：\n";
    std::cout << "  双手: 0-5  左手: l0-l5  右手: r0-r5\n";
    std::cout << "  3 = 双手应用左右各自实测任务动作；l3/r3 = 单独设置左/右手实测任务动作\n";
    for (const auto& hp : kHandPresets) {
        std::cout << "  " << hp.key << " = " << hp.name << "\n";
    }
    std::cout << "说明：本程序会发布 rt/lowcmd + 灵巧手 DDS，只能在调试模式下使用。\n";
}

// ===== 34. 主函数 =====
// 整体流程：
// 1. 初始化 DDS。
// 2. 订阅 rt/lowstate。
// 3. 创建 rt/lowcmd 发布器，但不立刻发布。
// 4. 输入 r 后才进入低刚度示教记录。
int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);

    const std::string network = (argc >= 2) ? argv[1] : kDefaultNetwork;

    std::cout << "R1 dual-arm + dexterous hand compliant teaching recorder\n";
    std::cout << "Network interface: " << network << "\n";
    std::cout << "Subscribe topic: " << kLowStateTopic << "\n";
    std::cout << "Publish topic: " << kLowCmdTopic << "\n";
    std::cout << "Hand topics: " << kLeftHandTopic << ", " << kRightHandTopic << "\n";
    std::cout << "Safety: publishes rt/lowcmd only after you type r; hand DDS publishes current gesture.\n";
    std::cout << "Mode: legs/waist locked stand + head disabled + low-stiffness arms + hand DDS.\n";

    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, network);

        unitree::robot::ChannelSubscriberPtr<LowState> lowstate_subscriber;
        lowstate_subscriber.reset(
            new unitree::robot::ChannelSubscriber<LowState>(kLowStateTopic));
        lowstate_subscriber->InitChannel(LowStateHandler, 1);

        unitree::robot::ChannelPublisher<LowCmd> lowcmd_publisher(kLowCmdTopic);
        lowcmd_publisher.InitChannel();

        unitree::robot::ChannelPublisher<HandCmdsMsg> left_hand_pub(kLeftHandTopic);
        left_hand_pub.InitChannel();
        unitree::robot::ChannelPublisher<HandCmdsMsg> right_hand_pub(kRightHandTopic);
        right_hand_pub.InitChannel();

        std::cout << "Waiting for first lowstate frame...\n";
        while (g_running.load()) {
            FullPose pose{};
            uint8_t mode_machine = 1;
            uint64_t seq = 0;
            if (GetLatestFullPose(pose, mode_machine, seq)) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (!g_running.load()) {
            std::cout << "\nExit before first lowstate frame.\n";
            return 0;
        }

        std::thread writer_thread(CommandWriterLoop, &lowcmd_publisher);
        std::thread hand_writer_thread(HandWriterLoop, &left_hand_pub, &right_hand_pub);

        PrintMenu();
        std::cout << "> " << std::flush;
        std::string input;
        while (g_running.load() && std::getline(std::cin, input)) {
            if (input.empty()) { PrintMenu(); goto prompt; }

            // ---- 多字符命令优先判断（必须在单字符之前）----

            // hh: 查看灵巧手姿态
            if (input == "hh" || input == "HH") {
                PrintDualHandPose(GetCurrentDualHandPose(), "当前灵巧手姿态");
                goto prompt;
            }

            // 0-5 / l0-l5 / r0-r5：设置灵巧手手势。
            // 这套解析函数也被录制过程复用，保证菜单操作和 JSON 记录规则一致。
            if (ApplyHandCommand(input, true)) {
                goto prompt;
            }

            // ---- 单字符命令（放在独立作用域内，避免 goto 跨越初始化）----
            {
                const char key = input.at(0);

                if (key == 'v' || key == 'V') {
                    FullPose pose{};
                    uint8_t mode_machine = 1;
                    uint64_t seq = 0;
                    if (GetLatestFullPose(pose, mode_machine, seq)) {
                        PrintDualArmPose(pose, "当前双臂角度", seq);
                    } else {
                        std::cout << "[WARN] 还没有收到 rt/lowstate。\n";
                    }
                } else if (key == 'r' || key == 'R') {
                    RecordMotion(lowcmd_publisher);
                } else if (key == 'h' || key == 'H') {
                    PrintMenu();
                } else if (key == 'q' || key == 'Q') {
                    break;
                } else {
                    std::cout << "无效输入。v/h/hh/q | 0-5(双手) | l0-l5(左手) | r0-r5(右手)。\n";
                }
            }

        prompt:
            if (g_running.load()) std::cout << "> " << std::flush;
        }

        g_control_enabled.store(false);
        PublishDisableFrames(lowcmd_publisher, 20);
        g_running.store(false);
        if (writer_thread.joinable()) writer_thread.join();
        if (hand_writer_thread.joinable()) hand_writer_thread.join();
    } catch (const std::exception& e) {
        std::cerr << "Program error: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "Unknown program error.\n";
        return 1;
    }

    std::cout << "Done.\n";
    return 0;
}
