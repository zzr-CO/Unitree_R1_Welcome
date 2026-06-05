/*
 * R1 full-body motor state recorder.
 *
 * Purpose:
 *   Record all usable R1 motor states while the robot is in locked standing
 *   posture. This program is read-only. It subscribes to rt/lowstate and does
 *   not publish rt/lowcmd, rt/arm_sdk, or any other control command.
 *
 * Compile on R1 from ~/unitree_sdk2-1.0:
 *   g++ -std=c++17 test_r1_full_body_state_recorder.cpp \
 *       -I./include -I./thirdparty/include/ddscxx \
 *       -L./lib/aarch64 -L./thirdparty/lib/aarch64 \
 *       -lunitree_sdk2 -lddscxx -lddsc -lpthread \
 *       -Wl,-rpath,./lib/aarch64:./thirdparty/lib/aarch64 \
 *       -o test_r1_full_body_state_recorder
 *
 * Run:
 *   ./test_r1_full_body_state_recorder eth10
 *
 * 学习目标：
 *   1. 理解“锁定站立参考值”：这是机器人站稳时全身电机的一组状态快照。
 *   2. 理解“只读记录”和“控制程序”的区别：本程序没有 ChannelPublisher，也没有 Write() 控制命令。
 *   3. 理解“平均采样”：同一个姿态下 q/dq/tau 会有轻微波动，采样 1 秒求平均更适合作参考。
 *   4. 为后续调试模式下的全身初始姿态、右臂动作安全检查做数据准备。
 *
 * 你可以改：
 *   - kDefaultNetwork：默认 DDS 网口。
 *   - kRecordDurationMs：每次记录采样多久。
 *   - kOutputFileName：保存文件名。
 *   - kR1Joints 里的中文名称，方便你自己理解。
 *
 * 暂时不要改：
 *   - kLowStateTopic = "rt/lowstate"。
 *   - kR1Joints 里的 idl_index，除非你确认官方关节顺序变化。
 *   - Crc32Core()，这是底层状态消息 CRC 校验。
 *
 * 安全说明：
 *   本程序只订阅状态，不发布任何电机控制命令。
 *   它可以在锁定站立/走跑运控状态下读取状态，但不要和危险底层控制程序混淆。
 */

// ===== 1. C++ 标准库头文件 =====
// 你应该学会：
// - array/vector：保存固定关节表和多帧采样。
// - atomic：处理 Ctrl+C 退出。
// - fstream：把记录结果写入文本文件。
// - mutex：保护 DDS 回调线程和主线程共享的状态。
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

// ===== 2. Unitree SDK2 头文件 =====
// LowState_ 是 R1 底层状态消息，里面包含全身电机状态。
// ChannelSubscriber 是 DDS 订阅器，用来接收 rt/lowstate。
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

// ===== 3. 类型别名 =====
// 你应该学会：using 能把很长的类型名变短。
using LowState = unitree_hg::msg::dds_::LowState_;

// ===== 4. 全局常量 =====
// 你可以改：默认网口、记录时长、输出文件名。
// 暂时不要改：rt/lowstate topic。
static constexpr const char* kLowStateTopic = "rt/lowstate";
static constexpr const char* kDefaultNetwork = "eth10";
static constexpr const char* kOutputFileName = "r1_locked_stand_full_body_snapshot.txt";
static constexpr int kRecordDurationMs = 1000;
static constexpr int kRecordPollMs = 10;

// ===== 5. 运行开关 =====
// Ctrl+C 时会变成 false，主循环退出。
static std::atomic<bool> g_running{true};

// ===== 6. 关节描述结构体 =====
// idl_index：LowState 消息中的真实电机编号。
// english_name：官方英文名称。
// chinese_name：中文说明，方便现场看表。
// group_name：身体部位分组。
struct JointInfo {
    int idl_index;
    const char* english_name;
    const char* chinese_name;
    const char* group_name;
};

// ===== 7. R1 全身有效电机表 =====
// 这里列出 R1 26 个有效电机：
// - 0-13：腿和腰
// - 15-19：左臂
// - 22-26：右臂
// - 29-30：头部
// 文档里 14、20、21、27、28 是 EMPTY，所以不记录。
// 你可以改中文名；暂时不要改 IDL 编号。
static const std::array<JointInfo, 26> kR1Joints{{
    {0,  "L_LEG_HIP_PITCH",     "左髋前后",   "左腿"},
    {1,  "L_LEG_HIP_ROLL",      "左髋左右",   "左腿"},
    {2,  "L_LEG_HIP_YAW",       "左髋旋转",   "左腿"},
    {3,  "L_LEG_KNEE",          "左膝",       "左腿"},
    {4,  "L_LEG_ANKLE_PITCH",   "左踝前后",   "左腿"},
    {5,  "L_LEG_ANKLE_ROLL",    "左踝左右",   "左腿"},
    {6,  "R_LEG_HIP_PITCH",     "右髋前后",   "右腿"},
    {7,  "R_LEG_HIP_ROLL",      "右髋左右",   "右腿"},
    {8,  "R_LEG_HIP_YAW",       "右髋旋转",   "右腿"},
    {9,  "R_LEG_KNEE",          "右膝",       "右腿"},
    {10, "R_LEG_ANKLE_PITCH",   "右踝前后",   "右腿"},
    {11, "R_LEG_ANKLE_ROLL",    "右踝左右",   "右腿"},
    {12, "WAIST_ROLL",          "腰左右倾斜", "腰部"},
    {13, "WAIST_YAW",           "腰部旋转",   "腰部"},
    {15, "L_SHOULDER_PITCH",    "左肩前后",   "左臂"},
    {16, "L_SHOULDER_ROLL",     "左肩左右",   "左臂"},
    {17, "L_SHOULDER_YAW",      "左肩旋转",   "左臂"},
    {18, "L_ELBOW",             "左肘",       "左臂"},
    {19, "L_WRIST_ROLL",        "左腕旋转",   "左臂"},
    {22, "R_SHOULDER_PITCH",    "右肩前后",   "右臂"},
    {23, "R_SHOULDER_ROLL",     "右肩左右",   "右臂"},
    {24, "R_SHOULDER_YAW",      "右肩旋转",   "右臂"},
    {25, "R_ELBOW",             "右肘",       "右臂"},
    {26, "R_WRIST_ROLL",        "右腕旋转",   "右臂"},
    {29, "HEAD_PITCH",          "头部俯仰",   "头部"},
    {30, "HEAD_YAW",            "头部左右转", "头部"},
}};

// ===== 8. 单个电机状态 =====
// q：位置，单位 rad。
// dq：速度。
// tau_est：估计力矩。
// mode：电机模式。
// motor_state：电机状态码，正常一般为 0。
// temperature：电机温度数组的两个值。
// voltage：电机电压相关读数。
struct MotorSample {
    float q = 0.0f;
    float dq = 0.0f;
    float tau_est = 0.0f;
    uint8_t mode = 0;
    uint8_t motor_state = 0;
    uint8_t temperature_0 = 0;
    uint8_t temperature_1 = 0;
    float voltage = 0.0f;
};

// ===== 9. 一帧全身状态快照 =====
// seq：本程序收到的有效帧序号。
// mode_machine：机器人当前模式字段，后续判断状态时可能有用。
// samples：26 个有效电机的状态。
struct FullBodySnapshot {
    std::array<MotorSample, kR1Joints.size()> samples{};
    uint8_t mode_machine = 0;
    uint64_t seq = 0;
    bool valid = false;
};

// ===== 10. 跨线程共享状态 =====
// DDS 回调线程写入最新状态，主线程读取并打印/记录。
// 多线程共享变量必须加锁。
static std::mutex g_snapshot_mutex;
static FullBodySnapshot g_latest_snapshot;

// ===== 11. Ctrl+C 信号处理 =====
// 只负责让程序退出，不做控制命令。
static void SignalHandler(int) {
    g_running.store(false);
}

// ===== 12. CRC 校验函数 =====
// 可以理解为“数据封条”：确认收到的 LowState 没损坏。
// 这段来自 Unitree 底层示例，暂时不要改。
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
// 控制和记录建议保留 rad；现场阅读时 deg 更直观。
static float RadToDeg(float rad) {
    return rad * 180.0f / 3.14159265358979323846f;
}

// ===== 14. 从 LowState 读取一个电机 =====
// 你应该学会：底层消息中每个电机都有一组状态量，不只有角度 q。
static MotorSample ReadMotor(const LowState& state, int idl_index) {
    const auto& motor = state.motor_state().at(idl_index);
    MotorSample sample;
    sample.q = motor.q();
    sample.dq = motor.dq();
    sample.tau_est = motor.tau_est();
    sample.mode = motor.mode();
    sample.motor_state = motor.motorstate();
    sample.temperature_0 = motor.temperature()[0];
    sample.temperature_1 = motor.temperature()[1];
    sample.voltage = motor.vol();
    return sample;
}

// ===== 15. LowState 回调函数 =====
// 每收到一帧 rt/lowstate，SDK 自动调用这里。
// 这个函数只读取，不控制。
static void LowStateHandler(const void* message) {
    LowState state = *(const LowState*)message;

    const uint32_t expected_crc =
        Crc32Core((uint32_t*)&state, (sizeof(LowState) >> 2) - 1);
    if (state.crc() != expected_crc) {
        std::cerr << "[WARN] lowstate CRC mismatch, skip this frame\n";
        return;
    }

    FullBodySnapshot snapshot;
    for (size_t i = 0; i < kR1Joints.size(); ++i) {
        snapshot.samples.at(i) = ReadMotor(state, kR1Joints.at(i).idl_index);
    }
    snapshot.mode_machine = state.mode_machine();
    snapshot.valid = true;

    std::lock_guard<std::mutex> lock(g_snapshot_mutex);
    snapshot.seq = g_latest_snapshot.seq + 1;
    g_latest_snapshot = snapshot;
}

// ===== 16. 获取最新快照 =====
// 返回一份拷贝，避免主线程直接操作共享变量。
static FullBodySnapshot GetLatestSnapshot() {
    std::lock_guard<std::mutex> lock(g_snapshot_mutex);
    return g_latest_snapshot;
}

// ===== 17. 采样累加结构 =====
// 用 double 累加多帧数据，最后除以帧数得到平均值。
struct MotorAccumulator {
    double q = 0.0;
    double dq = 0.0;
    double tau_est = 0.0;
    double voltage = 0.0;
    uint8_t mode = 0;
    uint8_t motor_state = 0;
    uint8_t temperature_0 = 0;
    uint8_t temperature_1 = 0;
};

// ===== 18. 对锁定站立状态采样求平均 =====
// 输入 s 后调用。采样 kRecordDurationMs 毫秒，最后返回平均快照。
// 你可以改 kRecordDurationMs，让平均时间更长或更短。
static bool RecordAverageSnapshot(FullBodySnapshot& averaged, size_t& frame_count) {
    std::array<MotorAccumulator, kR1Joints.size()> acc{};
    frame_count = 0;
    uint8_t last_mode_machine = 0;
    uint64_t last_seq = 0;

    const auto start_time = std::chrono::steady_clock::now();
    const auto duration = std::chrono::milliseconds(kRecordDurationMs);

    while (g_running.load() && std::chrono::steady_clock::now() - start_time < duration) {
        const FullBodySnapshot snapshot = GetLatestSnapshot();
        if (!snapshot.valid || snapshot.seq == last_seq) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kRecordPollMs));
            continue;
        }

        last_seq = snapshot.seq;
        last_mode_machine = snapshot.mode_machine;
        ++frame_count;

        for (size_t i = 0; i < kR1Joints.size(); ++i) {
            const MotorSample& sample = snapshot.samples.at(i);
            acc.at(i).q += sample.q;
            acc.at(i).dq += sample.dq;
            acc.at(i).tau_est += sample.tau_est;
            acc.at(i).voltage += sample.voltage;
            acc.at(i).mode = sample.mode;
            acc.at(i).motor_state = sample.motor_state;
            acc.at(i).temperature_0 = sample.temperature_0;
            acc.at(i).temperature_1 = sample.temperature_1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kRecordPollMs));
    }

    if (frame_count == 0) return false;

    averaged = FullBodySnapshot{};
    averaged.valid = true;
    averaged.seq = last_seq;
    averaged.mode_machine = last_mode_machine;
    for (size_t i = 0; i < kR1Joints.size(); ++i) {
        MotorSample sample;
        sample.q = static_cast<float>(acc.at(i).q / frame_count);
        sample.dq = static_cast<float>(acc.at(i).dq / frame_count);
        sample.tau_est = static_cast<float>(acc.at(i).tau_est / frame_count);
        sample.voltage = static_cast<float>(acc.at(i).voltage / frame_count);
        sample.mode = acc.at(i).mode;
        sample.motor_state = acc.at(i).motor_state;
        sample.temperature_0 = acc.at(i).temperature_0;
        sample.temperature_1 = acc.at(i).temperature_1;
        averaged.samples.at(i) = sample;
    }
    return true;
}

// ===== 19. 打印全身状态表 =====
// 你可以通过 q(rad)/q(deg) 看站立姿态，通过 dq 看是否基本静止。
// 如果 motor_state 不是 0，要优先排查对应电机。
static void PrintSnapshotTable(std::ostream& out,
                               const FullBodySnapshot& snapshot,
                               size_t frame_count) {
    out << "\n================ R1 Full-Body Locked Stand Snapshot ================\n";
    out << "seq: " << snapshot.seq
        << " | mode_machine: " << static_cast<int>(snapshot.mode_machine)
        << " | averaged frames: " << frame_count
        << " | read-only: no command publisher\n";
    out << "--------------------------------------------------------------------\n";
    out << std::left
        << std::setw(5) << "IDL"
        << std::setw(8) << "部位"
        << std::setw(14) << "关节"
        << std::setw(22) << "英文代号"
        << std::right
        << std::setw(10) << "q(rad)"
        << std::setw(10) << "q(deg)"
        << std::setw(10) << "dq"
        << std::setw(10) << "tau"
        << std::setw(7) << "mode"
        << std::setw(8) << "state"
        << std::setw(9) << "temp"
        << std::setw(9) << "vol"
        << "\n";

    out << std::fixed << std::setprecision(3);
    for (size_t i = 0; i < kR1Joints.size(); ++i) {
        const JointInfo& joint = kR1Joints.at(i);
        const MotorSample& sample = snapshot.samples.at(i);
        std::ostringstream temp;
        temp << static_cast<int>(sample.temperature_0)
             << "/" << static_cast<int>(sample.temperature_1);

        out << std::left
            << std::setw(5) << joint.idl_index
            << std::setw(8) << joint.group_name
            << std::setw(14) << joint.chinese_name
            << std::setw(22) << joint.english_name
            << std::right
            << std::setw(10) << sample.q
            << std::setw(10) << RadToDeg(sample.q)
            << std::setw(10) << sample.dq
            << std::setw(10) << sample.tau_est
            << std::setw(7) << static_cast<int>(sample.mode)
            << std::setw(8) << static_cast<int>(sample.motor_state)
            << std::setw(9) << temp.str()
            << std::setw(9) << sample.voltage
            << "\n";
    }
    out << std::defaultfloat;
}

// ===== 20. 打印 C++ 站立角度数组 =====
// 后续如果要把锁定站立作为参考姿态，可以复制这里的 q(rad)。
// 注意：这不是完整站立控制器，只是“参考角度表”。
static void PrintCppPoseArray(std::ostream& out, const FullBodySnapshot& snapshot) {
    out << "\n// C++ reference pose, q(rad), order follows kR1Joints.\n";
    out << "static const std::array<float, " << kR1Joints.size()
        << "> kLockedStandPose = {\n    ";
    out << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < kR1Joints.size(); ++i) {
        out << snapshot.samples.at(i).q << "f";
        if (i + 1 < kR1Joints.size()) out << ", ";
        if ((i + 1) % 4 == 0 && i + 1 < kR1Joints.size()) out << "\n    ";
    }
    out << "\n};\n";
    out << std::defaultfloat;
}

// ===== 21. 保存记录文件 =====
// 文件会保存在你运行程序的当前目录，比如 ~/unitree_sdk2-1.0。
// 你可以改 kOutputFileName。
static bool SaveSnapshotToFile(const FullBodySnapshot& snapshot, size_t frame_count) {
    std::ofstream file(kOutputFileName);
    if (!file) return false;

    file << "R1 locked standing full-body motor snapshot\n";
    file << "Output file: " << kOutputFileName << "\n";
    file << "Record duration ms: " << kRecordDurationMs << "\n";
    PrintSnapshotTable(file, snapshot, frame_count);
    PrintCppPoseArray(file, snapshot);
    return true;
}

// ===== 22. 打印菜单 =====
// 程序启动后不会自动记录，必须手动输入 s。
static void PrintMenu() {
    std::cout << "\n================ R1 全身电机状态记录器 ================\n";
    std::cout << "  v = 查看当前最新一帧全身电机状态\n";
    std::cout << "  s = 采样 " << kRecordDurationMs << " ms，求平均并保存\n";
    std::cout << "  h = 显示菜单\n";
    std::cout << "  q = 退出\n";
    std::cout << "  Ctrl+C = 退出\n";
    std::cout << "> " << std::flush;
}

// ===== 23. 主函数入口 =====
// 整体流程：
// 1. 初始化 DDS。
// 2. 订阅 rt/lowstate。
// 3. 等待第一帧状态。
// 4. 等你输入 s 后采样并保存。
int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    const std::string network = (argc >= 2) ? argv[1] : kDefaultNetwork;

    std::cout << "R1 full-body motor state recorder\n";
    std::cout << "Network interface: " << network << "\n";
    std::cout << "Subscribe topic: " << kLowStateTopic << "\n";
    std::cout << "Output file: " << kOutputFileName << "\n";
    std::cout << "Safety: read-only, no ChannelPublisher, no motor command.\n";

    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, network);

        unitree::robot::ChannelSubscriberPtr<LowState> lowstate_subscriber;
        lowstate_subscriber.reset(
            new unitree::robot::ChannelSubscriber<LowState>(kLowStateTopic));
        lowstate_subscriber->InitChannel(LowStateHandler, 1);

        std::cout << "Waiting for first lowstate frame...\n";
        while (g_running.load() && !GetLatestSnapshot().valid) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!g_running.load()) {
            std::cout << "\nExit before first lowstate frame.\n";
            return 0;
        }

        PrintMenu();
        std::string input;
        while (g_running.load() && std::getline(std::cin, input)) {
            if (input.empty()) {
                PrintMenu();
                continue;
            }

            const char key = input.at(0);
            if (key == 'v' || key == 'V') {
                const FullBodySnapshot snapshot = GetLatestSnapshot();
                PrintSnapshotTable(std::cout, snapshot, 1);
            } else if (key == 's' || key == 'S') {
                FullBodySnapshot averaged;
                size_t frame_count = 0;
                std::cout << "Recording locked standing state for "
                          << kRecordDurationMs << " ms...\n";
                if (!RecordAverageSnapshot(averaged, frame_count)) {
                    std::cout << "[WARN] 没有采到有效 lowstate 帧。\n";
                } else {
                    PrintSnapshotTable(std::cout, averaged, frame_count);
                    PrintCppPoseArray(std::cout, averaged);
                    if (SaveSnapshotToFile(averaged, frame_count)) {
                        std::cout << "\nSaved to " << kOutputFileName << "\n";
                    } else {
                        std::cout << "\n[WARN] 保存文件失败，请检查当前目录权限。\n";
                    }
                }
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
    } catch (const std::exception& e) {
        std::cerr << "Program error: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "Unknown program error.\n";
        return 1;
    }

    std::cout << "\nR1 full-body motor state recorder exited.\n";
    return 0;
}
