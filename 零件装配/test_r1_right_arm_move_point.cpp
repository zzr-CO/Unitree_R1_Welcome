/*
 * R1 right arm teach-point motion test.
 *
 * This is task 5 of the low-level arm development path.
 * It reads the current right-arm posture from rt/lowstate. Each time you type
 * "s", it publishes rt/lowcmd and moves the right arm to the next recorded
 * teach point. This lets you test the assembly flow one small stage at a time.
 *
 * Compile on R1 from ~/unitree_sdk2-1.0:
 *   g++ -std=c++17 test_r1_right_arm_move_point.cpp \
 *       -I./include -I./thirdparty/include/ddscxx \
 *       -L./lib/aarch64 -L./thirdparty/lib/aarch64 \
 *       -lunitree_sdk2 -lddscxx -lddsc -lpthread \
 *       -Wl,-rpath,./lib/aarch64:./thirdparty/lib/aarch64 \
 *       -o test_r1_right_arm_move_point
 *
 * Run:
 *   ./test_r1_right_arm_move_point eth10
 *
 * 学习目标：
 *   1. 理解“示教点”：先手动摆出一个位置并记录角度，再让程序复现这个位置。
 *   2. 理解“多关节插值”：右肩、右肘、右腕一起从当前角度平滑移动到目标角度。
 *   3. 理解“分阶段流程”：每按一次 s 只执行一个小阶段，方便现场观察和调试。
 *   4. 理解“底层命令持续发布”：rt/lowcmd 必须周期性发布，不能只发一帧。
 *
 * 你可以改：
 *   - kDefaultNetwork：默认 DDS 网口。
 *   - kTargetPoint：这次要去的示教点，目前是 PART_ABOVE 零件上方。
 *   - r1_right_arm_teach_points.hpp：右臂点位表，后续优化点位主要改那里。
 *   - kMoveDurationMs：移动速度，数值越大动作越慢。
 *   - kHoldDurationMs：到达目标点后的停留时间。
 *   - kHoldKp / kHoldKd：右臂保持刚度和阻尼。先小后大，不要激进。
 *
 * 暂时不要改：
 *   - kLowCmdTopic = "rt/lowcmd"。
 *   - kLowStateTopic = "rt/lowstate"。
 *   - 右臂关节 ID：22-26。
 *   - Crc32Core()，这是底层消息 CRC 校验。
 *
 * 安全说明：
 *   本程序会在你输入 s 之后发布 rt/lowcmd。
 *   它只给右臂 22-26 设置 mode=1，其他关节 mode=0。
 *   如果机器人出现异常抖动、异响、明显阻力或姿态不合理，立刻按 x 或 Ctrl+C。
 */

// ===== 1. C++ 标准库头文件 =====
// 你应该学会：
// - atomic：跨线程安全控制程序运行、动作中断、命令发布。
// - thread：一个线程持续发布底层命令，一个线程执行动作流程。
// - mutex：保护最新姿态和目标姿态，避免两个线程同时读写。
// - array：保存右臂 5 个关节角度。
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

// ===== 2. Unitree SDK2 头文件 =====
// LowState_ 用来读取机器人当前底层状态。
// LowCmd_ 用来发送底层电机控制命令。
// ChannelSubscriber 是 DDS 接收者。
// ChannelPublisher 是 DDS 发送者。
#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

// ===== 3. 项目内右臂示教点文件 =====
// 你可以改：示教点数值放在这个头文件里。
// 你应该学会：点位数据和运动控制逻辑分开，后续维护更轻松。
#include "r1_right_arm_teach_points.hpp"

// ===== 4. 类型别名 =====
// 你应该学会：using 可以把长类型名缩短，代码更容易读。
using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using LowState = unitree_hg::msg::dds_::LowState_;
using ArmPose = RightArmPose;

// ===== 5. 全局常量 =====
// 你可以改：kDefaultNetwork、kTargetPoint、动作时间、kp/kd。
// 暂时不要改：topic、右臂关节 ID。
static constexpr const char* kLowCmdTopic = "rt/lowcmd";
static constexpr const char* kLowStateTopic = "rt/lowstate";
static constexpr const char* kDefaultNetwork = "eth10";
static constexpr int kControlPeriodUs = 2000;  // 2000us = 500Hz
static constexpr int kMotionUpdateMs = 10;     // 100Hz 更新插值目标
static constexpr int kPreHoldMs = 1000;
static constexpr int kMoveDurationMs = 4500;
static constexpr int kHoldDurationMs = 1200;
static constexpr float kHoldKp = 20.0f;
static constexpr float kHoldKd = 2.0f;
static constexpr float kZero = 0.0f;

// ===== 6. 任务 5 目标示教点 =====
// 你可以改：把 kTargetPoint 换成其他已记录点位。
// 当前目标是 PART_ABOVE：零件上方点。
// 可选点位都在 r1_right_arm_teach_points.hpp 里。
static const RightArmTeachPoint& kTargetPoint = kPartAbove;

static std::atomic<bool> g_running{true};
static std::atomic<bool> g_control_enabled{false};
static std::atomic<bool> g_motion_running{false};
static std::atomic<bool> g_abort_motion{false};
static size_t g_next_flow_step_index = 0;

// ===== 7. 右臂关节定义 =====
// idl_index 是 LowState/LowCmd 里的真实关节编号。
// 这一步的意义：让你看到“数组第几个值”对应“机器人哪个关节”。
struct JointInfo {
    int idl_index;
    const char* english_name;
    const char* chinese_name;
};

static const std::array<JointInfo, 5> kRightArmJoints{{
    {22, "R_SHOULDER_PITCH", "右肩前后"},
    {23, "R_SHOULDER_ROLL",  "右肩左右"},
    {24, "R_SHOULDER_YAW",   "右肩旋转"},
    {25, "R_ELBOW",          "右肘"},
    {26, "R_WRIST_ROLL",     "右腕旋转"},
}};

// ===== 8. 状态缓存 =====
// latest_pose：DDS 回调读到的右臂实时角度。
// command_pose：发布线程当前要发送的右臂目标角度。
// mode_machine：LowCmd 需要带上的机器人模式字段。
static std::mutex g_pose_mutex;
static ArmPose g_latest_pose{};
static ArmPose g_command_pose{};
static ArmPose g_last_flow_command_pose{};
static bool g_has_last_flow_command_pose = false;
static std::atomic<bool> g_pose_valid{false};
static uint8_t g_mode_machine = 0;
static uint64_t g_pose_seq = 0;

// ===== 9. Ctrl+C 信号处理 =====
// Ctrl+C 会中断动作、停止发布控制，然后退出。
static void SignalHandler(int) {
    g_abort_motion.store(true);
    g_control_enabled.store(false);
    g_running.store(false);
}

// ===== 10. CRC 校验函数 =====
// 暂时不要改：这是官方底层示例的常见写法。
// LowState 接收和 LowCmd 发送都需要 CRC。
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

// ===== 11. 弧度转角度 =====
// 控制用 rad，显示给人看时用 deg。
static float RadToDeg(float rad) {
    return rad * 180.0f / 3.14159265358979323846f;
}

// ===== 12. LowState 回调函数 =====
// 每收到一帧 rt/lowstate，SDK 会自动调用这个函数。
// 它只读取右臂当前 q，不发布任何命令。
static void LowStateHandler(const void* message) {
    LowState state = *(const LowState*)message;

    const uint32_t expected_crc =
        Crc32Core((uint32_t*)&state, (sizeof(LowState) >> 2) - 1);
    if (state.crc() != expected_crc) {
        std::cerr << "[WARN] lowstate CRC mismatch, skip this frame\n";
        return;
    }

    ArmPose pose{};
    for (size_t i = 0; i < kRightArmJoints.size(); ++i) {
        pose.at(i) = state.motor_state().at(kRightArmJoints.at(i).idl_index).q();
    }

    std::lock_guard<std::mutex> lock(g_pose_mutex);
    g_latest_pose = pose;
    g_mode_machine = state.mode_machine();
    g_pose_valid.store(true);
    ++g_pose_seq;
}

// ===== 13. 读取当前右臂姿态 =====
// 返回最新右臂姿态的一份拷贝。
static bool GetLatestPose(ArmPose& pose, uint64_t& seq) {
    std::lock_guard<std::mutex> lock(g_pose_mutex);
    if (!g_pose_valid.load()) return false;
    pose = g_latest_pose;
    seq = g_pose_seq;
    return true;
}

// ===== 14. 设置发布目标姿态 =====
// 动作线程通过这个函数更新目标角，发布线程会持续发送它。
static void SetCommandPose(const ArmPose& pose) {
    std::lock_guard<std::mutex> lock(g_pose_mutex);
    g_command_pose = pose;
}

// ===== 15. 记录分阶段流程的上一个命令目标 =====
// 你应该学会：
// - 机器人真实角度会因为重力、刚度参数等原因和命令目标有误差。
// - 分阶段动作的“下一段起点”应该优先使用上一段命令目标，而不是已经下垂的实际角度。
// - 这样按第二次 s 时，程序不会先命令手臂去偏低位置。
static void RememberLastFlowCommandPose(const ArmPose& pose) {
    std::lock_guard<std::mutex> lock(g_pose_mutex);
    g_last_flow_command_pose = pose;
    g_has_last_flow_command_pose = true;
}

static bool GetLastFlowCommandPose(ArmPose& pose) {
    std::lock_guard<std::mutex> lock(g_pose_mutex);
    if (!g_has_last_flow_command_pose) return false;
    pose = g_last_flow_command_pose;
    return true;
}

static void ClearLastFlowCommandPose() {
    std::lock_guard<std::mutex> lock(g_pose_mutex);
    g_has_last_flow_command_pose = false;
}

// ===== 16. 读取发布目标和 mode_machine =====
// 发布线程每 2ms 调用一次，用来组装 LowCmd。
static void GetCommandPoseAndMode(ArmPose& pose, uint8_t& mode_machine) {
    std::lock_guard<std::mutex> lock(g_pose_mutex);
    pose = g_command_pose;
    mode_machine = g_mode_machine;
}

// ===== 17. 打印右臂角度 =====
// 你可以改：setprecision(3) 显示更多或更少小数。
static void PrintRightArmPose(const ArmPose& pose, const char* title, uint64_t seq = 0) {
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
    for (size_t i = 0; i < kRightArmJoints.size(); ++i) {
        const auto& joint = kRightArmJoints.at(i);
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

// ===== 18. 打印本次动作变化量 =====
// 这能帮助你判断动作幅度大不大。
// 如果某个关节变化量太大，第一次测试时要更谨慎。
static void PrintPoseDelta(const ArmPose& from, const ArmPose& to) {
    std::cout << "\n本次动作变化量\n";
    std::cout << "----------------------------------------------------------------\n";
    std::cout << std::left
              << std::setw(5) << "IDL"
              << std::setw(14) << "关节"
              << std::setw(20) << "英文代号"
              << std::right
              << std::setw(12) << "delta(rad)"
              << std::setw(12) << "delta(deg)"
              << "\n";

    std::cout << std::fixed << std::setprecision(3);
    for (size_t i = 0; i < kRightArmJoints.size(); ++i) {
        const auto& joint = kRightArmJoints.at(i);
        const float delta = to.at(i) - from.at(i);
        std::cout << std::left
                  << std::setw(5) << joint.idl_index
                  << std::setw(14) << joint.chinese_name
                  << std::setw(20) << joint.english_name
                  << std::right
                  << std::setw(12) << delta
                  << std::setw(12) << RadToDeg(delta)
                  << "\n";
    }
    std::cout << std::defaultfloat;
}

// ===== 19. 清空 LowCmd =====
// 默认禁用所有关节，后续只启用右臂 5 个关节。
// 关键安全点：腿、腰、头、左臂都保持 mode=0。
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

// ===== 20. 填充右臂目标命令 =====
// 只有右臂关节 mode=1。
// command_pose 是动作线程计算出来的当前插值目标。
static void FillRightArmCommand(LowCmd& cmd, const ArmPose& command_pose) {
    for (size_t i = 0; i < kRightArmJoints.size(); ++i) {
        const int id = kRightArmJoints.at(i).idl_index;
        cmd.motor_cmd().at(id).mode() = 1;
        cmd.motor_cmd().at(id).tau() = kZero;
        cmd.motor_cmd().at(id).q() = command_pose.at(i);
        cmd.motor_cmd().at(id).dq() = kZero;
        cmd.motor_cmd().at(id).kp() = kHoldKp;
        cmd.motor_cmd().at(id).kd() = kHoldKd;
    }
}

// ===== 21. 发送 disable 帧 =====
// 停止动作或退出时调用，连续发送几帧全部 disabled 的命令。
static void PublishDisableFrames(unitree::robot::ChannelPublisher<LowCmd>& publisher,
                                 int frame_count) {
    ArmPose unused_pose{};
    uint8_t mode_machine = 0;
    GetCommandPoseAndMode(unused_pose, mode_machine);

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

// ===== 22. LowCmd 发布线程 =====
// g_control_enabled=true 时持续发布右臂目标角。
// 底层控制通常需要周期发送，不能只发一次。
static void CommandWriterLoop(unitree::robot::ChannelPublisher<LowCmd>* publisher) {
    while (g_running.load()) {
        if (!g_control_enabled.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        ArmPose command_pose{};
        uint8_t mode_machine = 0;
        GetCommandPoseAndMode(command_pose, mode_machine);

        LowCmd cmd;
        DisableAllMotors(cmd);
        cmd.mode_pr() = 0;
        cmd.mode_machine() = mode_machine;
        FillRightArmCommand(cmd, command_pose);
        cmd.crc() = Crc32Core((uint32_t*)&cmd, (sizeof(cmd) >> 2) - 1);

        publisher->Write(cmd);
        std::this_thread::sleep_for(std::chrono::microseconds(kControlPeriodUs));
    }
}

// ===== 23. 插值函数 =====
// ratio 从 0 到 1，输出 start 到 end 之间的平滑中间值。
// 你应该学会：机器人动作不要跳变，要一点点插过去。
static float Lerp(float start, float end, float ratio) {
    return start + (end - start) * ratio;
}

// ===== 24. 平滑比例函数 =====
// 这是一个简单的 ease-in/ease-out，让动作起步和停止更柔和。
// 输入 ratio: 0 到 1，输出仍然是 0 到 1。
static float SmoothRatio(float ratio) {
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    return ratio * ratio * (3.0f - 2.0f * ratio);
}

// ===== 25. 保持某个姿态一段时间 =====
// 这段不会改变目标角，只是持续让发布线程保持当前目标。
static void HoldPoseFor(const ArmPose& pose, int duration_ms) {
    SetCommandPose(pose);
    const auto start = std::chrono::steady_clock::now();
    const auto duration = std::chrono::milliseconds(duration_ms);
    while (g_running.load() && !g_abort_motion.load() &&
           std::chrono::steady_clock::now() - start < duration) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kMotionUpdateMs));
    }
}

// ===== 26. 从一个姿态平滑移动到另一个姿态 =====
// 这里五个右臂关节都会按照同一个比例插值。
// 你可以理解为：五个旋钮同时慢慢拧到目标刻度。
static void MovePose(const ArmPose& from, const ArmPose& to, int duration_ms) {
    const auto start = std::chrono::steady_clock::now();
    const auto duration = std::chrono::milliseconds(duration_ms);

    while (g_running.load() && !g_abort_motion.load()) {
        const auto elapsed = std::chrono::steady_clock::now() - start;
        float ratio = static_cast<float>(
            std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()) /
            static_cast<float>(duration_ms);
        if (ratio > 1.0f) ratio = 1.0f;

        const float smooth = SmoothRatio(ratio);
        ArmPose current{};
        for (size_t i = 0; i < current.size(); ++i) {
            current.at(i) = Lerp(from.at(i), to.at(i), smooth);
        }
        SetCommandPose(current);

        if (elapsed >= duration) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(kMotionUpdateMs));
    }
}

// ===== 27. 移动到 PART_ABOVE 再返回的动作流程 =====
// 流程：
// 1. 读取当前右臂姿态作为 start_pose。
// 2. 打印 start_pose、PART_ABOVE 和变化量。
// 3. 保持当前姿态 1 秒。
// 4. 平滑移动到 PART_ABOVE。
// 5. 停留 1.2 秒。
// 6. 平滑回到 start_pose。
// 7. 停止发布控制。
static void RunMoveToPointAndBack() {
    if (g_motion_running.exchange(true)) {
        std::cout << "动作已经在运行中。\n";
        return;
    }

    ArmPose start_pose{};
    uint64_t seq = 0;
    if (!GetLatestPose(start_pose, seq)) {
        std::cout << "[WARN] 还没有收到 rt/lowstate，不能开始动作。\n";
        g_motion_running.store(false);
        return;
    }

    PrintRightArmPose(start_pose, "起始右臂角度", seq);
    PrintRightArmPose(kTargetPoint.pose, kTargetPoint.name);
    PrintPoseDelta(start_pose, kTargetPoint.pose);

    std::cout << "\n动作开始：当前姿态 -> " << kTargetPoint.name
              << " -> 返回当前姿态。\n";
    std::cout << "移动时间：" << kMoveDurationMs
              << " ms，停留时间：" << kHoldDurationMs << " ms。\n";

    g_abort_motion.store(false);
    g_control_enabled.store(true);

    HoldPoseFor(start_pose, kPreHoldMs);
    MovePose(start_pose, kTargetPoint.pose, kMoveDurationMs);
    HoldPoseFor(kTargetPoint.pose, kHoldDurationMs);
    MovePose(kTargetPoint.pose, start_pose, kMoveDurationMs);
    HoldPoseFor(start_pose, kPreHoldMs);

    g_control_enabled.store(false);
    if (g_abort_motion.load()) {
        std::cout << "动作已中断。\n";
    } else {
        std::cout << "动作完成：右臂已返回动作开始前的姿态。\n";
    }
    g_motion_running.store(false);
}

// 交互式启动动作，避免程序一运行就控制机器人。
// ===== 28. 分阶段流程步骤定义 =====
// 你可以改：每一步的目标点和停留时间。
// 暂时不要改：先不要在这里加入灵巧手，当前程序只验证右臂点位。
// 你应该学会：复杂动作可以拆成多个小阶段，每按一次 s 只执行一个阶段。
struct MotionStep {
    const RightArmTeachPoint* point;
    int hold_ms;
};

static const std::array<MotionStep, 6> kStepFlow{{
    {&kPartAbove, 1000},
    {&kPartGrasp, 1000},
    {&kPartLift, 1000},
    {&kAssemblyAbove, 1000},
    {&kAssemblyInsert, 1000},
    {&kReturnHome, 1200},
}};

// ===== 29. 打印分阶段流程 =====
// 用来确认下一次按 s 会走到哪个点。
static void PrintStepFlow() {
    std::cout << "\n分阶段右臂流程：\n";
    for (size_t i = 0; i < kStepFlow.size(); ++i) {
        const MotionStep& step = kStepFlow.at(i);
        const bool is_next = (i == g_next_flow_step_index);
        std::cout << "  " << (is_next ? "-> " : "   ")
                  << (i + 1) << ". " << step.point->name
                  << "  hold=" << step.hold_ms << "ms\n";
    }
    if (g_next_flow_step_index >= kStepFlow.size()) {
        std::cout << "  -> 流程已完成，输入 r 可重置到第一步。\n";
    }
}

// ===== 30. 执行下一阶段 =====
// 每次输入 s，只移动到下一阶段目标点。
// 第一阶段从当前实际右臂姿态出发。
// 后续阶段从“上一阶段命令目标”出发，避免实际保持误差导致下一段动作先下落。
// 执行成功后，下一次 s 会自动进入下一个阶段。
static void RunNextFlowStep() {
    if (g_motion_running.exchange(true)) {
        std::cout << "动作已经在运行中。\n";
        return;
    }

    if (g_next_flow_step_index >= kStepFlow.size()) {
        std::cout << "分阶段流程已经完成。输入 r 重置后可重新执行。\n";
        g_motion_running.store(false);
        return;
    }

    ArmPose actual_start_pose{};
    uint64_t seq = 0;
    if (!GetLatestPose(actual_start_pose, seq)) {
        std::cout << "[WARN] 还没有收到 rt/lowstate，不能开始下一阶段。\n";
        g_motion_running.store(false);
        return;
    }

    ArmPose command_start_pose = actual_start_pose;
    const bool has_previous_command = GetLastFlowCommandPose(command_start_pose);

    const size_t step_index = g_next_flow_step_index;
    const MotionStep& step = kStepFlow.at(step_index);

    std::cout << "\n执行阶段 " << (step_index + 1) << "/"
              << kStepFlow.size() << "：" << step.point->name << "\n";
    PrintRightArmPose(actual_start_pose, "阶段起始右臂实际角度", seq);
    if (has_previous_command) {
        PrintRightArmPose(command_start_pose, "本阶段命令起点（上一阶段目标）");
        std::cout << "说明：实际角度如果比命令起点低，通常是保持刚度偏软或负载导致的下垂。\n";
    }
    PrintRightArmPose(step.point->pose, step.point->name);
    PrintPoseDelta(command_start_pose, step.point->pose);
    std::cout << "移动时间：" << kMoveDurationMs
              << " ms，停留时间：" << step.hold_ms << " ms。\n";

    g_abort_motion.store(false);
    g_control_enabled.store(true);

    HoldPoseFor(command_start_pose, has_previous_command ? 200 : kPreHoldMs);
    MovePose(command_start_pose, step.point->pose, kMoveDurationMs);
    HoldPoseFor(step.point->pose, step.hold_ms);

    if (g_abort_motion.load()) {
        g_control_enabled.store(false);
        ClearLastFlowCommandPose();
        std::cout << "阶段已中断，流程序号不前进。\n";
    } else {
        ++g_next_flow_step_index;
        SetCommandPose(step.point->pose);
        RememberLastFlowCommandPose(step.point->pose);
        g_control_enabled.store(true);
        std::cout << "阶段完成。右臂将继续保持在当前目标点。下一次 s 将执行下一阶段。\n";
    }
    g_motion_running.store(false);
}

// ===== 31. 重置分阶段流程 =====
// 输入 r 后，下一次 s 又会从 PART_ABOVE 开始。
static void ResetStepFlow() {
    g_next_flow_step_index = 0;
    ClearLastFlowCommandPose();
    std::cout << "分阶段流程已重置，下一次 s 将移动到 "
              << kStepFlow.at(g_next_flow_step_index).point->name << "。\n";
}

static void PrintMenu() {
    std::cout << "\n================ 右臂示教点移动测试 ================\n";
    std::cout << "目标点：" << kTargetPoint.name << "\n";
    std::cout << "  v = 查看当前右臂角度\n";
    std::cout << "  t = 查看目标点角度\n";
    std::cout << "  p = 查看分阶段流程和下一步\n";
    std::cout << "  s = 执行下一阶段，只走一个点位\n";
    std::cout << "  r = 重置分阶段流程到第一步\n";
    std::cout << "  x = 中断动作并停止发布控制\n";
    std::cout << "  h = 显示菜单\n";
    std::cout << "  q = 退出\n";
    std::cout << "  Ctrl+C = 中断动作并退出\n";
    std::cout << "> " << std::flush;
}

// ===== 28. 主函数入口 =====
// 整体流程：
// 1. 初始化 DDS。
// 2. 订阅 rt/lowstate。
// 3. 创建 rt/lowcmd 发布器，但不立刻发布。
// 4. 等待用户输入 s。
// 5. 每次 s 只执行下一阶段，直到走完整个装配示教流程。
int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    const std::string network = (argc >= 2) ? argv[1] : kDefaultNetwork;

    std::cout << "R1 right arm teach-point motion test\n";
    std::cout << "Network interface: " << network << "\n";
    std::cout << "Subscribe topic: " << kLowStateTopic << "\n";
    std::cout << "Publish topic: " << kLowCmdTopic << "\n";
    std::cout << "Safety: publishes rt/lowcmd only after you type s.\n";
    std::cout << "Right arm only: IDL 22-26.\n";

    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, network);

        unitree::robot::ChannelSubscriberPtr<LowState> lowstate_subscriber;
        lowstate_subscriber.reset(
            new unitree::robot::ChannelSubscriber<LowState>(kLowStateTopic));
        lowstate_subscriber->InitChannel(LowStateHandler, 1);

        unitree::robot::ChannelPublisher<LowCmd> lowcmd_publisher(kLowCmdTopic);
        lowcmd_publisher.InitChannel();

        std::cout << "Waiting for first lowstate frame...\n";
        while (g_running.load() && !g_pose_valid.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!g_running.load()) {
            std::cout << "\nExit before first lowstate frame.\n";
            return 0;
        }

        std::thread writer_thread(CommandWriterLoop, &lowcmd_publisher);
        std::thread motion_thread;

        PrintMenu();
        std::string input;
        while (g_running.load() && std::getline(std::cin, input)) {
            if (input.empty()) {
                PrintMenu();
                continue;
            }

            const char key = input.at(0);
            if (key == 'v' || key == 'V') {
                ArmPose pose{};
                uint64_t seq = 0;
                if (GetLatestPose(pose, seq)) {
                    PrintRightArmPose(pose, "当前右臂角度", seq);
                } else {
                    std::cout << "[WARN] 还没有收到 rt/lowstate。\n";
                }
            } else if (key == 't' || key == 'T') {
                PrintRightArmPose(kTargetPoint.pose, kTargetPoint.name);
            } else if (key == 'p' || key == 'P') {
                PrintStepFlow();
            } else if (key == 's' || key == 'S') {
                if (g_motion_running.load()) {
                    std::cout << "动作已经在运行中，输入 x 可中断。\n";
                } else {
                    if (motion_thread.joinable()) motion_thread.join();
                    motion_thread = std::thread(RunNextFlowStep);
                }
            } else if (key == 'r' || key == 'R') {
                if (g_motion_running.load()) {
                    std::cout << "动作运行中，不能重置。先输入 x 中断。\n";
                } else {
                    ResetStepFlow();
                }
            } else if (key == 'x' || key == 'X') {
                g_abort_motion.store(true);
                g_control_enabled.store(false);
                ClearLastFlowCommandPose();
                PublishDisableFrames(lowcmd_publisher, 20);
                std::cout << "已中断动作并发送 disable 帧。\n";
            } else if (key == 'h' || key == 'H') {
                PrintMenu();
            } else if (key == 'q' || key == 'Q') {
                break;
            } else {
                std::cout << "无效输入。输入 h 查看菜单。\n";
            }

            if (g_running.load()) {
                std::cout << "\n> " << std::flush;
            }
        }

        g_abort_motion.store(true);
        g_control_enabled.store(false);
        ClearLastFlowCommandPose();
        if (motion_thread.joinable()) motion_thread.join();
        PublishDisableFrames(lowcmd_publisher, 20);
        g_running.store(false);
        if (writer_thread.joinable()) writer_thread.join();
    } catch (const std::exception& e) {
        std::cerr << "Program error: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "Unknown program error.\n";
        return 1;
    }

    std::cout << "\nR1 right arm teach-point motion test exited.\n";
    return 0;
}
