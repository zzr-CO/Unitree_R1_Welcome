/*
 * R1 dual-arm + dexterous-hand trajectory replayer.
 * GitHub display label: R1 teaching replay program.
 *
 * Purpose:
 *   Play back a dual-arm trajectory JSON file recorded by
 *   test_r1_teach_record_motion.cpp. The robot must be in debug mode and
 *   hung securely — this program publishes rt/lowcmd and BrainCo hand DDS.
 *
 * Compile on R1 from ~/unitree_sdk2-1.0:
 *   g++ -std=c++17 test_r1_dual_arm_replay.cpp \
 *       -I./include -I./thirdparty/include/ddscxx \
 *       -L./lib/aarch64 -L./thirdparty/lib/aarch64 \
 *       -lunitree_sdk2 -lddscxx -lddsc -lpthread \
 *       -Wl,-rpath,./lib/aarch64:./thirdparty/lib/aarch64 \
 *       -o test_r1_dual_arm_replay
 *
 * Run:
 *   ./test_r1_dual_arm_replay eth10 R1-hand.json
 *
 * 你可以改：
 *   - JSON 文件名（第二个命令行参数）。
 *   - kPlaySpeed：回放速度，1.0=原速，0.5=半速。
 *   - kStandKp/kStandKd：腿腰保持刚度。
 *   - kArmKp/kArmKd：手臂回放刚度。
 *
 * 暂时不要改：
 *   - topic、关节 ID、CRC。
 *   - kLockedStandPose 里的值。
 *
 * 安全说明：
 *   本程序会发布 rt/lowcmd 和灵巧手 DDS。
 *   腿/腰：命令到锁定站立基准。
 *   头部：不检查、不保持、不发布命令。
 *   左右臂和双手：按 JSON 轨迹回放。
 *   如果出现异常立刻按 x 或 Ctrl+C。
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/idl/go2/MotorCmds_.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using LowState = unitree_hg::msg::dds_::LowState_;
using FullPose = std::array<float, 26>;
using DualArmPose = std::array<float, 10>;
using HandPose = std::array<float, 6>;
using DualHandPose = std::array<float, 12>;
using HandCmdsMsg = unitree_go::msg::dds_::MotorCmds_;

// ===== DDS 参数 =====
static constexpr const char* kLowCmdTopic = "rt/lowcmd";
static constexpr const char* kLowStateTopic = "rt/lowstate";
static constexpr const char* kLeftHandTopic = "rt/brainco/left/cmd";
static constexpr const char* kRightHandTopic = "rt/brainco/right/cmd";
static constexpr const char* kDefaultNetwork = "eth10";
static constexpr const char* kDefaultJsonPath = "R1-hand.json";
static constexpr int kControlPeriodUs = 2000;
static constexpr float kZero = 0.0f;

// ===== 回放参数（你可以改）=====
static constexpr float kPlaySpeed = 1.0f;     // 1.0 = 原速
static constexpr float kStandKp = 90.0f;      // 腿腰站立刚度。保持和示教程序一致
static constexpr float kStandKd = 5.0f;       // 腿腰站立阻尼。保持和示教程序一致
static constexpr float kArmKp = 55.0f;        // 手臂回放刚度。点位偏差大时先加 Kp
static constexpr float kArmKd = 5.0f;         // 手臂回放阻尼。Kp 变大后适当加 Kd 防抖
static constexpr float kHandSpeed = 1.0f;     // 灵巧手速度
static constexpr int kHandPublishPeriodMs = 100;
static constexpr double kInitialAlignSec = 3.0;

// ===== 运行开关 =====
static std::atomic<bool> g_running{true};
static std::atomic<bool> g_control_enabled{false};
static std::atomic<bool> g_abort{false};
static std::atomic<bool> g_motion_running{false};

// ===== R1 26 电机表（与 teach_record 一致）=====
struct JointInfo {
    int idl_index;
    const char* english_name;
    const char* group_name;
};

static const std::array<JointInfo, 26> kR1Joints{{
    {0,  "L_LEG_HIP_PITCH",   "左腿"},
    {1,  "L_LEG_HIP_ROLL",    "左腿"},
    {2,  "L_LEG_HIP_YAW",     "左腿"},
    {3,  "L_LEG_KNEE",        "左腿"},
    {4,  "L_LEG_ANKLE_PITCH", "左腿"},
    {5,  "L_LEG_ANKLE_ROLL",  "左腿"},
    {6,  "R_LEG_HIP_PITCH",   "右腿"},
    {7,  "R_LEG_HIP_ROLL",    "右腿"},
    {8,  "R_LEG_HIP_YAW",     "右腿"},
    {9,  "R_LEG_KNEE",        "右腿"},
    {10, "R_LEG_ANKLE_PITCH", "右腿"},
    {11, "R_LEG_ANKLE_ROLL",  "右腿"},
    {12, "WAIST_ROLL",        "腰部"},
    {13, "WAIST_YAW",         "腰部"},
    {15, "L_SHOULDER_PITCH",  "左臂"},
    {16, "L_SHOULDER_ROLL",   "左臂"},
    {17, "L_SHOULDER_YAW",    "左臂"},
    {18, "L_ELBOW",           "左臂"},
    {19, "L_WRIST_ROLL",      "左臂"},
    {22, "R_SHOULDER_PITCH",  "右臂"},
    {23, "R_SHOULDER_ROLL",   "右臂"},
    {24, "R_SHOULDER_YAW",    "右臂"},
    {25, "R_ELBOW",           "右臂"},
    {26, "R_WRIST_ROLL",      "右臂"},
    {29, "HEAD_PITCH",        "头部"},
    {30, "HEAD_YAW",          "头部"},
}};

// 左右臂在 26 电机表中的下标
static const std::array<size_t, 5> kLeftArmIndices{{14, 15, 16, 17, 18}};
static const std::array<size_t, 5> kRightArmIndices{{19, 20, 21, 22, 23}};

// ===== 锁定站立基准（来自全身记录器 100 帧平均）=====
static const FullPose kLockedStandPose = {
    -0.285902f, 0.049563f, -0.148795f, 0.425323f,
    -0.184718f, -0.055339f, -0.324874f, -0.043069f,
    0.139767f, 0.419463f, -0.151288f, 0.067168f,
    -0.003439f, -0.000778f, 0.199532f, 0.180349f,
    -0.000526f, 0.810018f, -0.000011f, 0.201036f,
    -0.186743f, 0.003484f, 0.809403f, -0.001595f,
    0.003146f, 0.001092f
};

// ===== 轨迹帧 =====
struct TrajectoryFrame {
    double t;
    DualArmPose q;
    DualHandPose h;
};

// ===== 共享状态 =====
static std::mutex g_state_mutex;
static FullPose g_latest_full_pose{};
static uint8_t g_mode_machine = 1;
static bool g_state_valid = false;

static std::mutex g_cmd_mutex;
static FullPose g_command_pose = kLockedStandPose;

static std::atomic<bool> g_hand_enabled{false};
static std::mutex g_hand_mutex;
static HandPose g_left_hand_pose{};
static HandPose g_right_hand_pose{};

// ===== CRC =====
static uint32_t Crc32Core(uint32_t* ptr, uint32_t len) {
    uint32_t crc32 = 0xFFFFFFFF;
    const uint32_t polynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        uint32_t xbit = 1U << 31;
        uint32_t data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (crc32 & 0x80000000) crc32 = (crc32 << 1) ^ polynomial;
            else crc32 <<= 1;
            if (data & xbit) crc32 ^= polynomial;
            xbit >>= 1;
        }
    }
    return crc32;
}

// ===== 信号处理 =====
static void SignalHandler(int) {
    g_abort.store(true);
    g_control_enabled.store(false);
    g_running.store(false);
}

// ===== LowState 回调 =====
static void LowStateHandler(const void* message) {
    LowState state = *(const LowState*)message;
    const uint32_t expected_crc =
        Crc32Core((uint32_t*)&state, (sizeof(LowState) >> 2) - 1);
    if (state.crc() != expected_crc) return;

    FullPose pose{};
    for (size_t i = 0; i < kR1Joints.size(); ++i) {
        pose.at(i) = state.motor_state().at(kR1Joints.at(i).idl_index).q();
    }

    std::lock_guard<std::mutex> lock(g_state_mutex);
    g_latest_full_pose = pose;
    g_mode_machine = state.mode_machine();
    g_state_valid = true;
}

// ===== JSON 解析（手动，不依赖第三方库）=====
// 从 JSON 字符串中提取一个 double 值
static size_t ParseDouble(const std::string& text, size_t start, double& out) {
    size_t end = start;
    while (end < text.size() && text[end] != ',' && text[end] != ']' &&
           text[end] != '}' && text[end] != ' ' && text[end] != '\n') {
        ++end;
    }
    out = std::stod(text.substr(start, end - start));
    return end;
}

// 解析一帧: {"t": xxx, "q": [...], "h": [...]}
static bool ParseFrame(const std::string& text, size_t& pos, TrajectoryFrame& frame) {
    frame.h.fill(0.0f);

    // 找 "t":
    size_t t_key = text.find("\"t\":", pos);
    if (t_key == std::string::npos) return false;
    pos = t_key + 4;
    // 跳过空格
    while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\n')) ++pos;
    pos = ParseDouble(text, pos, frame.t);

    // 找 "q": [
    size_t q_key = text.find("\"q\":", pos);
    if (q_key == std::string::npos) return false;
    pos = text.find('[', q_key);
    if (pos == std::string::npos) return false;
    ++pos;

    for (size_t i = 0; i < 10; ++i) {
        while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\n')) ++pos;
        double val = 0.0;
        pos = ParseDouble(text, pos, val);
        frame.q.at(i) = static_cast<float>(val);
        while (pos < text.size() && text[pos] != ',' && text[pos] != ']') ++pos;
        if (text[pos] == ',') ++pos;
    }
    // 跳过 ]
    while (pos < text.size() && text[pos] != ']') ++pos;
    ++pos;

    // 找可选的 "h": [ ... ]。旧 JSON 没有 h 时默认双手张开。
    const size_t frame_end = text.find('}', pos);
    size_t h_key = text.find("\"h\":", pos);
    if (h_key != std::string::npos &&
        (frame_end == std::string::npos || h_key < frame_end)) {
        pos = text.find('[', h_key);
        if (pos == std::string::npos) return false;
        ++pos;

        for (size_t i = 0; i < frame.h.size(); ++i) {
            while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\n')) ++pos;
            double val = 0.0;
            pos = ParseDouble(text, pos, val);
            frame.h.at(i) = static_cast<float>(val);
            while (pos < text.size() && text[pos] != ',' && text[pos] != ']') ++pos;
            if (text[pos] == ',') ++pos;
        }
        while (pos < text.size() && text[pos] != ']') ++pos;
        ++pos;
    }

    if (frame_end != std::string::npos && pos < frame_end) {
        pos = frame_end + 1;
    }
    return true;
}

// 加载全部轨迹帧
static std::vector<TrajectoryFrame> LoadTrajectory(const std::string& filepath) {
    std::vector<TrajectoryFrame> frames;

    std::ifstream in(filepath);
    if (!in) {
        std::cerr << "[ERROR] 无法打开文件: " << filepath << "\n";
        return frames;
    }

    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string text = buffer.str();

    // 找 "frames": [
    size_t frames_key = text.find("\"frames\":");
    if (frames_key == std::string::npos) {
        std::cerr << "[ERROR] 找不到 \"frames\" 字段\n";
        return frames;
    }
    size_t pos = text.find('[', frames_key);
    if (pos == std::string::npos) {
        std::cerr << "[ERROR] 找不到 frames 数组\n";
        return frames;
    }
    ++pos;

    while (true) {
        TrajectoryFrame frame{};
        if (!ParseFrame(text, pos, frame)) break;
        frames.push_back(frame);
        if (pos >= text.size()) break;
    }

    return frames;
}

// ===== LowCmd 组装 =====
static void SetCommandPose(const FullPose& pose) {
    std::lock_guard<std::mutex> lock(g_cmd_mutex);
    g_command_pose = pose;
}

static FullPose GetCommandPose() {
    std::lock_guard<std::mutex> lock(g_cmd_mutex);
    return g_command_pose;
}

static uint8_t GetModeMachine() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    return g_mode_machine;
}

static bool GetLatestFullPose(FullPose& pose) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (!g_state_valid) return false;
    pose = g_latest_full_pose;
    return true;
}

static DualArmPose ExtractDualArm(const FullPose& pose) {
    DualArmPose dual{};
    for (size_t i = 0; i < kLeftArmIndices.size(); ++i) {
        dual.at(i) = pose.at(kLeftArmIndices.at(i));
        dual.at(i + 5) = pose.at(kRightArmIndices.at(i));
    }
    return dual;
}

static bool IsHeadJoint(const JointInfo& joint) {
    return std::strcmp(joint.group_name, "头部") == 0;
}

// 把双臂 10 关节写入全身 cmd_pose，腿/腰用 kLockedStandPose，头部不控制
static FullPose BuildFullPoseFromDualArm(const DualArmPose& arm_q) {
    FullPose cmd = kLockedStandPose;
    for (size_t i = 0; i < 5; ++i) {
        cmd.at(kLeftArmIndices.at(i)) = arm_q.at(i);
        cmd.at(kRightArmIndices.at(i)) = arm_q.at(i + 5);
    }
    return cmd;
}

static void SplitDualHandPose(const DualHandPose& dual, HandPose& left, HandPose& right) {
    for (size_t i = 0; i < 6; ++i) {
        left.at(i) = dual.at(i);
        right.at(i) = dual.at(i + 6);
    }
}

static void SetCurrentHandPose(const DualHandPose& dual) {
    HandPose left{}, right{};
    SplitDualHandPose(dual, left, right);

    std::lock_guard<std::mutex> lock(g_hand_mutex);
    g_left_hand_pose = left;
    g_right_hand_pose = right;
}

static void GetCurrentHandPoses(HandPose& left, HandPose& right) {
    std::lock_guard<std::mutex> lock(g_hand_mutex);
    left = g_left_hand_pose;
    right = g_right_hand_pose;
}

static void SetHandPose(HandCmdsMsg& msg, const HandPose& pose) {
    msg.cmds().resize(6);
    for (size_t i = 0; i < pose.size(); ++i) {
        msg.cmds()[i].q() = pose.at(i);
        msg.cmds()[i].dq() = kHandSpeed;
    }
}

static void HandWriterLoop(
    unitree::robot::ChannelPublisher<HandCmdsMsg>* left_pub,
    unitree::robot::ChannelPublisher<HandCmdsMsg>* right_pub) {
    while (g_running.load()) {
        if (!g_hand_enabled.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

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

// 填充 LowCmd
static void FillLowCmd(LowCmd& cmd, const FullPose& cmd_pose, uint8_t mode_machine) {
    // 先全部禁用
    for (size_t i = 0; i < cmd.motor_cmd().size(); ++i) {
        cmd.motor_cmd().at(i).mode() = 0;
        cmd.motor_cmd().at(i).tau() = kZero;
        cmd.motor_cmd().at(i).q() = kZero;
        cmd.motor_cmd().at(i).dq() = kZero;
        cmd.motor_cmd().at(i).kp() = kZero;
        cmd.motor_cmd().at(i).kd() = kZero;
    }

    // 按分组设置
    for (size_t i = 0; i < kR1Joints.size(); ++i) {
        const JointInfo& joint = kR1Joints.at(i);
        if (IsHeadJoint(joint)) {
            continue;
        }

        const int id = joint.idl_index;
        float kp, kd;
        const char* g = joint.group_name;

        if (std::strcmp(g, "左腿") == 0 || std::strcmp(g, "右腿") == 0 ||
            std::strcmp(g, "腰部") == 0) {
            kp = kStandKp; kd = kStandKd;
        } else {
            kp = kArmKp; kd = kArmKd;
        }

        cmd.motor_cmd().at(id).mode() = 1;
        cmd.motor_cmd().at(id).tau() = kZero;
        cmd.motor_cmd().at(id).q() = cmd_pose.at(i);
        cmd.motor_cmd().at(id).dq() = kZero;
        cmd.motor_cmd().at(id).kp() = kp;
        cmd.motor_cmd().at(id).kd() = kd;
    }
}

// 发送 disable 帧
static void PublishDisableFrames(unitree::robot::ChannelPublisher<LowCmd>& publisher,
                                  int count) {
    LowCmd cmd;
    for (size_t i = 0; i < cmd.motor_cmd().size(); ++i) {
        cmd.motor_cmd().at(i).mode() = 0;
        cmd.motor_cmd().at(i).tau() = kZero;
        cmd.motor_cmd().at(i).q() = kZero;
        cmd.motor_cmd().at(i).dq() = kZero;
        cmd.motor_cmd().at(i).kp() = kZero;
        cmd.motor_cmd().at(i).kd() = kZero;
    }
    cmd.mode_pr() = 0;
    cmd.mode_machine() = GetModeMachine();

    for (int i = 0; i < count; ++i) {
        cmd.crc() = Crc32Core((uint32_t*)&cmd, (sizeof(cmd) >> 2) - 1);
        publisher.Write(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

// ===== 命令发布线程 =====
static void CommandWriterLoop(unitree::robot::ChannelPublisher<LowCmd>* publisher) {
    while (g_running.load()) {
        if (!g_control_enabled.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        FullPose cmd_pose = GetCommandPose();
        uint8_t mode_machine = GetModeMachine();

        LowCmd cmd;
        cmd.mode_pr() = 0;
        cmd.mode_machine() = mode_machine;
        FillLowCmd(cmd, cmd_pose, mode_machine);
        cmd.crc() = Crc32Core((uint32_t*)&cmd, (sizeof(cmd) >> 2) - 1);
        publisher->Write(cmd);

        std::this_thread::sleep_for(std::chrono::microseconds(kControlPeriodUs));
    }
}

// ===== 打印双臂 =====
static void PrintDualArmPose(const DualArmPose& q, const char* title) {
    std::cout << "\n" << title << "\n";
    std::cout << "左臂: [";
    for (size_t i = 0; i < 5; ++i) {
        std::cout << std::fixed << std::setprecision(4) << q.at(i);
        if (i < 4) std::cout << ", ";
    }
    std::cout << "]\n右臂: [";
    for (size_t i = 0; i < 5; ++i) {
        std::cout << std::fixed << std::setprecision(4) << q.at(i + 5);
        if (i < 4) std::cout << ", ";
    }
    std::cout << "]\n" << std::defaultfloat;
}

// ===== 直线插值 =====
static DualArmPose InterpolateDualArm(const DualArmPose& from, const DualArmPose& to,
                                       float ratio) {
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    // smooth step
    float smooth = ratio * ratio * (3.0f - 2.0f * ratio);
    DualArmPose result{};
    for (size_t i = 0; i < 10; ++i) {
        result.at(i) = from.at(i) + (to.at(i) - from.at(i)) * smooth;
    }
    return result;
}

static DualHandPose InterpolateDualHand(const DualHandPose& from, const DualHandPose& to,
                                        float ratio) {
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    DualHandPose result{};
    for (size_t i = 0; i < result.size(); ++i) {
        result.at(i) = from.at(i) + (to.at(i) - from.at(i)) * ratio;
    }
    return result;
}

static void PrintDualHandPose(const DualHandPose& h, const char* title) {
    std::cout << "\n" << title << "\n";
    std::cout << "左手: [";
    for (size_t i = 0; i < 6; ++i) {
        std::cout << std::fixed << std::setprecision(2) << h.at(i);
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]\n右手: [";
    for (size_t i = 0; i < 6; ++i) {
        std::cout << std::fixed << std::setprecision(2) << h.at(i + 6);
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]\n" << std::defaultfloat;
}

// ===== 执行回放 =====
static void RunReplay(unitree::robot::ChannelPublisher<LowCmd>& publisher,
                      const std::vector<TrajectoryFrame>& frames) {
    if (g_motion_running.exchange(true)) {
        std::cout << "回放已经在运行中。\n";
        return;
    }

    if (frames.empty()) {
        std::cout << "[WARN] 没有轨迹帧，无法回放。\n";
        g_motion_running.store(false);
        return;
    }

    g_abort.store(false);

    std::cout << "\n回放 " << frames.size() << " 帧，"
              << "时长 " << frames.back().t << " 秒，"
              << "速度 " << kPlaySpeed << "x\n";

    PrintDualArmPose(frames.front().q, "起始姿态");
    PrintDualArmPose(frames.back().q, "结束姿态");
    PrintDualHandPose(frames.front().h, "起始灵巧手姿态");
    PrintDualHandPose(frames.back().h, "结束灵巧手姿态");

    FullPose latest_pose{};
    if (!GetLatestFullPose(latest_pose)) {
        std::cout << "[WARN] 还没有收到当前 lowstate，无法自动对齐第一帧。\n";
        g_motion_running.store(false);
        return;
    }

    const DualArmPose current_q = ExtractDualArm(latest_pose);
    const DualHandPose first_h = frames.front().h;
    SetCurrentHandPose(first_h);
    g_hand_enabled.store(true);
    g_control_enabled.store(true);

    std::cout << "自动平滑对齐到第一帧，耗时约 "
              << kInitialAlignSec << " 秒...\n";
    const auto align_start = std::chrono::steady_clock::now();
    while (g_running.load() && !g_abort.load()) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - align_start).count();
        const float ratio = static_cast<float>(elapsed / kInitialAlignSec);
        if (ratio >= 1.0f) {
            break;
        }

        const DualArmPose align_q = InterpolateDualArm(current_q, frames.front().q, ratio);
        SetCommandPose(BuildFullPoseFromDualArm(align_q));
        SetCurrentHandPose(first_h);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (g_abort.load() || !g_running.load()) {
        PublishDisableFrames(publisher, 20);
        g_control_enabled.store(false);
        g_hand_enabled.store(false);
        g_motion_running.store(false);
        return;
    }

    SetCommandPose(BuildFullPoseFromDualArm(frames.front().q));
    SetCurrentHandPose(first_h);
    std::cout << "已到达第一帧，保持 1 秒后开始正式回放...\n";
    const auto hold_start = std::chrono::steady_clock::now();
    while (g_running.load() && !g_abort.load() &&
           std::chrono::steady_clock::now() - hold_start < std::chrono::seconds(1)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "开始回放轨迹...\n";
    auto replay_start = std::chrono::steady_clock::now();
    const double total_duration = frames.back().t / kPlaySpeed;
    size_t current_frame = 0;
    auto last_print = replay_start;

    while (g_running.load() && !g_abort.load()) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - replay_start).count();
        double play_time = elapsed * kPlaySpeed;

        if (elapsed >= total_duration) {
            // 到达最后一帧
            break;
        }

        // 找到当前时间对应的两帧
        while (current_frame + 1 < frames.size() &&
               frames.at(current_frame + 1).t <= play_time) {
            ++current_frame;
        }

        size_t next_frame = current_frame + 1;
        if (next_frame >= frames.size()) next_frame = current_frame;

        double t0 = frames.at(current_frame).t;
        double t1 = frames.at(next_frame).t;
        float ratio = (t1 > t0) ? static_cast<float>((play_time - t0) / (t1 - t0)) : 0.0f;

        DualArmPose interp_q = InterpolateDualArm(
            frames.at(current_frame).q, frames.at(next_frame).q, ratio);
        FullPose cmd_pose = BuildFullPoseFromDualArm(interp_q);
        SetCommandPose(cmd_pose);

        DualHandPose interp_h = InterpolateDualHand(
            frames.at(current_frame).h, frames.at(next_frame).h, ratio);
        SetCurrentHandPose(interp_h);

        // 每秒打印一次进度
        if (now - last_print > std::chrono::seconds(1)) {
            std::cout << "进度: " << std::fixed << std::setprecision(1)
                      << elapsed << "/" << total_duration << " 秒\n";
            last_print = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    std::cout << std::defaultfloat;
    if (g_abort.load()) {
        std::cout << "回放已中断。\n";
    } else {
        SetCommandPose(BuildFullPoseFromDualArm(frames.back().q));
        SetCurrentHandPose(frames.back().h);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        std::cout << "回放完成。\n";
    }

    // 发送 disable 帧
    PublishDisableFrames(publisher, 20);
    g_control_enabled.store(false);
    g_hand_enabled.store(false);
    g_motion_running.store(false);
    // 让命令姿态停在最后帧
    if (!frames.empty()) {
        SetCommandPose(BuildFullPoseFromDualArm(frames.back().q));
        SetCurrentHandPose(frames.back().h);
    }
}

// ===== 菜单 =====
static void PrintMenu(const std::vector<TrajectoryFrame>& frames,
                      const std::string& json_path) {
    std::cout << "\n================ R1 双臂+灵巧手轨迹回放 ================\n";
    std::cout << "JSON: " << json_path << "\n";
    std::cout << "轨迹帧数: " << frames.size()
              << " | 时长: " << (frames.empty() ? 0.0 : frames.back().t) << " 秒\n";
    std::cout << "回放速度: " << kPlaySpeed << "x\n";
    std::cout << "开始回放前会自动平滑对齐到第一帧，约 " << kInitialAlignSec << " 秒。\n";
    std::cout << "  v = 查看首末帧姿态\n";
    std::cout << "  r = 开始回放\n";
    std::cout << "  x = 中断\n";
    std::cout << "  q = 退出\n";
    std::cout << "说明：腿/腰保持锁定站立，头部不控制，双臂和灵巧手按轨迹运动。\n";
}

// ===== 主函数 =====
int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    const std::string network = (argc >= 2) ? argv[1] : kDefaultNetwork;
    const std::string json_path = (argc >= 3)
        ? argv[2]
        : kDefaultJsonPath;

    std::cout << "R1 dual-arm + dexterous-hand trajectory replayer\n";
    std::cout << "Network: " << network << "\n";
    std::cout << "JSON: " << json_path << "\n";
    std::cout << "Hand topics: " << kLeftHandTopic << ", " << kRightHandTopic << "\n";

    // 加载轨迹
    std::vector<TrajectoryFrame> frames = LoadTrajectory(json_path);
    if (frames.empty()) {
        std::cerr << "[ERROR] 轨迹为空或加载失败。\n";
        return 1;
    }
    std::cout << "Loaded " << frames.size() << " frames.\n";

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

        std::cout << "Waiting for lowstate...\n";
        while (g_running.load() && !g_state_valid) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!g_running.load()) { std::cout << "Exit.\n"; return 0; }

        std::thread writer_thread(CommandWriterLoop, &lowcmd_publisher);
        std::thread hand_writer_thread(HandWriterLoop, &left_hand_pub, &right_hand_pub);
        std::thread motion_thread;

        PrintMenu(frames, json_path);
        std::string input;
        while (g_running.load() && std::getline(std::cin, input)) {
            if (input.empty()) { PrintMenu(frames, json_path); continue; }
            const char key = input.at(0);

            if (key == 'v' || key == 'V') {
                PrintDualArmPose(frames.front().q, "起始帧双臂姿态");
                PrintDualArmPose(frames.back().q, "结束帧双臂姿态");
                PrintDualHandPose(frames.front().h, "起始帧双手姿态");
                PrintDualHandPose(frames.back().h, "结束帧双手姿态");
            } else if (key == 'r' || key == 'R') {
                if (g_motion_running.load()) {
                    std::cout << "回放已经在运行中，输入 x 可中断。\n";
                } else {
                    if (motion_thread.joinable()) motion_thread.join();
                    motion_thread = std::thread(RunReplay, std::ref(lowcmd_publisher),
                                                std::cref(frames));
                }
            } else if (key == 'x' || key == 'X') {
                g_abort.store(true);
                g_control_enabled.store(false);
                g_hand_enabled.store(false);
                PublishDisableFrames(lowcmd_publisher, 20);
                std::cout << "已中断。\n";
            } else if (key == 'q' || key == 'Q') {
                break;
            } else {
                std::cout << "无效输入。v/r/x/q\n";
            }
            std::cout << "> " << std::flush;
        }

        g_abort.store(true);
        g_control_enabled.store(false);
        g_hand_enabled.store(false);
        if (motion_thread.joinable()) motion_thread.join();
        PublishDisableFrames(lowcmd_publisher, 20);
        g_running.store(false);
        if (writer_thread.joinable()) writer_thread.join();
        if (hand_writer_thread.joinable()) hand_writer_thread.join();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "Done.\n";
    return 0;
}
