/*
 * R1 arm state reader.
 *
 * This is the first low-level motion development test for this project.
 * It only subscribes to rt/lowstate and prints arm joint states. It does not
 * publish rt/lowcmd and does not control any motor.
 *
 * Compile on R1 from ~/unitree_sdk2-1.0:
 *   g++ -std=c++17 test_r1_arm_state.cpp \
 *       -I./include -I./thirdparty/include/ddscxx \
 *       -L./lib/aarch64 -L./thirdparty/lib/aarch64 \
 *       -lunitree_sdk2 -lddscxx -lddsc -lpthread \
 *       -Wl,-rpath,./lib/aarch64:./thirdparty/lib/aarch64 \
 *       -o test_r1_arm_state
 *
 * Run:
 *   ./test_r1_arm_state eth10
 *
 * Tip:
 *   This program is read-only. It is safe to run before writing any low-level
 *   arm controller, but it still needs the correct DDS network interface.
 *
 * 学习目标：
 *   1. 看懂 R1 底层状态 topic：rt/lowstate。
 *   2. 看懂关节角 q、速度 dq、估计力矩 tau_est 这些基础状态量。
 *   3. 学会如何用 C++ 结构体、数组、回调函数、互斥锁保存机器人状态。
 *   4. 为后续“示教点位记录”和“底层手臂控制”做准备。
 *
 * 你可以改：
 *   - 默认网口 eth10，如果你的 R1 DDS 网口不是 eth10，可以改 main() 里的默认值。
 *   - kLeftArmJoints / kRightArmJoints 里的中文名称，方便你自己理解。
 *   - 主循环里的打印间隔，现在是 1 秒打印一次。
 *
 * 暂时不要改：
 *   - kLowStateTopic = "rt/lowstate"，这是 R1 底层状态话题。
 *   - Crc32Core()，这是校验 DDS 底层状态消息是否完整的函数。
 *   - LowStateHandler() 里的读取逻辑，除非你已经确认新的关节 ID。
 *
 * 安全说明：
 *   本程序没有 ChannelPublisher，也没有 Write() 电机命令。
 *   它只订阅状态，不发布 rt/lowcmd，因此不会主动控制机器人运动。
 */

// ===== 1. C++ 标准库头文件 =====
// 你应该学会：
// - array：固定长度数组，适合保存固定数量的关节。
// - atomic：跨线程安全的开关变量，这里用于 Ctrl+C 退出。
// - chrono/thread：做定时打印。
// - mutex：保护回调线程和主线程共享的数据。
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

// ===== 2. Unitree SDK2 头文件 =====
// LowState_ 是 R1 底层状态消息类型，里面包含所有关节电机状态。
// ChannelSubscriber 是 DDS 订阅器，用来接收 rt/lowstate。
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

// ===== 3. 类型别名 =====
// 这行只是给很长的 DDS 类型起一个短名字，后面代码更好读。
// 你应该学会：using 可以给复杂类型起别名。
using LowState = unitree_hg::msg::dds_::LowState_;

// ===== 4. 全局常量和运行开关 =====
// kLowStateTopic 是 R1 发布底层状态的 DDS 话题。
// 你暂时不要改这个 topic；我们现在只读状态，不读别的话题。
static constexpr const char* kLowStateTopic = "rt/lowstate";

// g_running 是程序运行开关。
// Ctrl+C 时 SignalHandler 会把它改成 false，主循环就会退出。
static std::atomic<bool> g_running{true};

// ===== 5. 关节描述结构体 =====
// JointInfo 保存“一个关节是谁”：
// - idl_index：这个关节在 R1 LowState 消息里的编号。
// - name：官方英文代号，方便和 SDK 文档对照。
// - cn_name：中文名，方便现场看表。
// 你可以改：cn_name 的中文描述。
// 不建议改：idl_index，除非你确认官方关节顺序。
struct JointInfo {
    int idl_index;
    const char* name;
    const char* cn_name;
};

// ===== 6. 单个关节的实时状态 =====
// JointSample 保存“一个关节此刻是什么状态”：
// - q：当前位置，单位是弧度 rad，后面会换算成角度 deg。
// - dq：当前速度。
// - tau_est：估计力矩，粗略理解为关节受力/输出力矩相关信息。
// - mode：电机模式。
// - motor_state：电机状态码。
// 你应该学会：struct 可以把相关数据打包在一起。
struct JointSample {
    float q = 0.0f;
    float dq = 0.0f;
    float tau_est = 0.0f;
    uint8_t mode = 0;
    uint8_t motor_state = 0;
};

// ===== 7. 一帧左右臂快照 =====
// ArmSnapshot 保存一次 rt/lowstate 回调里读到的左右臂数据。
// 这里每条手臂只读 5 个关节：
// 肩前后、肩左右、肩旋转、肘、腕旋转。
// valid 表示有没有收到过有效数据。
// seq 是序号，每收到一帧有效数据就加 1。
struct ArmSnapshot {
    std::array<JointSample, 5> left{};
    std::array<JointSample, 5> right{};
    bool valid = false;
    uint64_t seq = 0;
};

// ===== 8. 左臂关节表 =====
// 这里定义我们关心的左臂 5 个关节。
// 你可以改：中文名 cn_name，让它更符合你的理解。
// 暂时不要改：15-19 这些 IDL 编号，这是根据 R1 关节顺序来的。
static const std::array<JointInfo, 5> kLeftArmJoints{{
    {15, "L_SHOULDER_PITCH", "左肩前后"},
    {16, "L_SHOULDER_ROLL",  "左肩左右"},
    {17, "L_SHOULDER_YAW",   "左肩旋转"},
    {18, "L_ELBOW",          "左肘"},
    {19, "L_WRIST_ROLL",     "左腕旋转"},
}};

// ===== 9. 右臂关节表 =====
// 这里定义我们关心的右臂 5 个关节。
// 后续做抓取装配动作，优先从右臂这 5 个关节开始，不碰头、腰、腿。
static const std::array<JointInfo, 5> kRightArmJoints{{
    {22, "R_SHOULDER_PITCH", "右肩前后"},
    {23, "R_SHOULDER_ROLL",  "右肩左右"},
    {24, "R_SHOULDER_YAW",   "右肩旋转"},
    {25, "R_ELBOW",          "右肘"},
    {26, "R_WRIST_ROLL",     "右腕旋转"},
}};

// ===== 10. 跨线程共享数据 =====
// Unitree DDS 回调线程负责接收最新状态；main 主线程负责每秒打印。
// 这两个线程会同时访问 g_snapshot，所以必须用 mutex 保护。
// 你应该学会：多个线程共享变量时，不要直接裸读裸写。
static std::mutex g_snapshot_mutex;
static ArmSnapshot g_snapshot;

// ===== 11. Ctrl+C 信号处理 =====
// 当你按 Ctrl+C，系统会调用这个函数。
// 它只做一件事：把运行开关设为 false，让程序温和退出。
static void SignalHandler(int) {
    g_running = false;
}

// ===== 12. CRC 校验函数 =====
// CRC 可以理解成“快递封条”：用来确认收到的 rt/lowstate 数据没有损坏。
// 如果 CRC 不匹配，说明这一帧数据不可靠，程序会丢弃它。
// 你暂时不要改这个函数，它来自 Unitree 底层示例里的通用写法。
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
// R1 底层关节角 q 的单位是 rad（弧度）。
// 但人更容易看懂 deg（角度），所以打印时做一次换算。
// 你应该记住：180 度 = pi 弧度。
static float RadToDeg(float rad) {
    return rad * 180.0f / 3.14159265358979323846f;
}

// ===== 14. 从 LowState 里读取一个关节 =====
// 输入：
// - state：一整帧 R1 底层状态。
// - idl_index：要读哪个关节。
// 输出：
// - JointSample：这个关节的 q/dq/tau/mode/state。
// 后续如果要多读头部或腰部，也会从这里扩展。
static JointSample ReadJoint(const LowState& state, int idl_index) {
    const auto& motor = state.motor_state().at(idl_index);
    JointSample sample;
    sample.q = motor.q();
    sample.dq = motor.dq();
    sample.tau_est = motor.tau_est();
    sample.mode = motor.mode();
    sample.motor_state = motor.motorstate();
    return sample;
}

// ===== 15. DDS 状态回调函数 =====
// 每当 R1 在 rt/lowstate 发来一帧新状态，SDK 就会自动调用这个函数。
// 这个函数只做三件事：
// 1. 拷贝消息。
// 2. 做 CRC 校验。
// 3. 读取左右臂关节并保存到 g_snapshot。
// 你应该学会：回调函数不是 main 主动调用的，是 DDS 收到消息后自动触发的。
static void LowStateHandler(const void* message) {
    LowState state = *(const LowState*)message;

    // 先校验这一帧数据是否完整。不完整就直接跳过，避免打印错误状态。
    const uint32_t expected_crc =
        Crc32Core((uint32_t*)&state, (sizeof(LowState) >> 2) - 1);
    if (state.crc() != expected_crc) {
        std::cerr << "[WARN] lowstate CRC mismatch, skip this frame\n";
        return;
    }

    // 从这一帧 LowState 中提取左右臂各 5 个关节。
    ArmSnapshot snapshot;
    for (size_t i = 0; i < kLeftArmJoints.size(); ++i) {
        snapshot.left.at(i) = ReadJoint(state, kLeftArmJoints.at(i).idl_index);
        snapshot.right.at(i) = ReadJoint(state, kRightArmJoints.at(i).idl_index);
    }
    snapshot.valid = true;

    // 把新快照写入全局变量。
    // lock_guard 会自动加锁和解锁，避免回调线程和打印线程冲突。
    std::lock_guard<std::mutex> lock(g_snapshot_mutex);
    snapshot.seq = g_snapshot.seq + 1;
    g_snapshot = snapshot;
}

// ===== 16. 读取最新快照 =====
// main 主线程每秒调用一次这个函数，拿到最近一次有效关节状态。
// 注意：这里返回的是一份拷贝，不是直接返回全局变量本身。
static ArmSnapshot GetSnapshot() {
    std::lock_guard<std::mutex> lock(g_snapshot_mutex);
    return g_snapshot;
}

// ===== 17. 打印单条手臂状态表 =====
// 这个函数只负责把左臂或右臂 5 个关节打印成表格。
// 你可以改：
// - 表头文字。
// - 打印列的宽度 setw。
// - 是否显示 tau/mode/state。
// 你应该学会：std::setw 用来控制终端表格宽度。
static void PrintArm(const char* title,
                     const std::array<JointInfo, 5>& joints,
                     const std::array<JointSample, 5>& samples) {
    std::cout << "\n" << title << "\n";
    std::cout << "--------------------------------------------------------------------------\n";
    std::cout << std::left
              << std::setw(5) << "IDL"
              << std::setw(14) << "关节"
              << std::setw(20) << "英文代号"
              << std::right
              << std::setw(10) << "q(rad)"
              << std::setw(10) << "q(deg)"
              << std::setw(10) << "dq"
              << std::setw(10) << "tau"
              << std::setw(7) << "mode"
              << std::setw(7) << "state"
              << "\n";

    // fixed + setprecision(3)：所有小数固定保留 3 位，更方便观察角度变化。
    std::cout << std::fixed << std::setprecision(3);
    for (size_t i = 0; i < joints.size(); ++i) {
        const auto& joint = joints.at(i);
        const auto& sample = samples.at(i);

        // 每一行对应一个关节：
        // IDL 编号 + 中文名 + 官方英文代号 + 角度/速度/力矩/状态。
        std::cout << std::left
                  << std::setw(5) << joint.idl_index
                  << std::setw(14) << joint.cn_name
                  << std::setw(20) << joint.name
                  << std::right
                  << std::setw(10) << sample.q
                  << std::setw(10) << RadToDeg(sample.q)
                  << std::setw(10) << sample.dq
                  << std::setw(10) << sample.tau_est
                  << std::setw(7) << static_cast<int>(sample.mode)
                  << std::setw(7) << static_cast<int>(sample.motor_state)
                  << "\n";
    }
    std::cout << std::defaultfloat;
}

// ===== 18. 打印一整帧左右臂快照 =====
// PrintSnapshot 负责组织整体输出：
// - 先打印 seq 和 topic。
// - 再分别打印左臂、右臂。
// 你后续做“示教点位记录”时，可以在这里增加“一键复制点位数组”的输出。
static void PrintSnapshot(const ArmSnapshot& snapshot) {
    std::cout << "\n================ R1 Arm LowState Snapshot ================\n";
    std::cout << "seq: " << snapshot.seq
              << " | topic: " << kLowStateTopic
              << " | read-only: no rt/lowcmd publisher\n";
    PrintArm("Left arm / 左臂", kLeftArmJoints, snapshot.left);
    PrintArm("Right arm / 右臂", kRightArmJoints, snapshot.right);
    std::cout << "==========================================================\n";
}

// ===== 19. 主函数入口 =====
// 程序从 main() 开始执行。
// 整体流程：
// 1. 注册 Ctrl+C 退出函数。
// 2. 读取命令行网口参数，默认 eth10。
// 3. 初始化 Unitree DDS。
// 4. 订阅 rt/lowstate。
// 5. 每秒打印一次最新左右臂状态。
int main(int argc, char** argv) {
    // 注册退出信号。按 Ctrl+C 时，SignalHandler 会把 g_running 改成 false。
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    // 命令行参数：
    // ./test_r1_arm_state eth10
    // 如果你没有传 eth10，就默认使用 eth10。
    // 你可以改：默认网口 "eth10"，但更推荐运行时传参。
    const std::string network = (argc >= 2) ? argv[1] : "eth10";

    std::cout << "R1 arm state reader\n";
    std::cout << "Network interface: " << network << "\n";
    std::cout << "Subscribe topic: " << kLowStateTopic << "\n";
    std::cout << "Safety: this program only reads state; it does not publish rt/lowcmd.\n";
    std::cout << "Press Ctrl+C to exit.\n";

    try {
        // 初始化 DDS 通信。
        // 参数 0 是 domain id，network 是网口名，比如 eth10。
        unitree::robot::ChannelFactory::Instance()->Init(0, network);

        // 创建 rt/lowstate 订阅器。
        // InitChannel(LowStateHandler, 1) 的意思是：
        // 收到消息后调用 LowStateHandler，队列深度为 1，只保留最新帧。
        unitree::robot::ChannelSubscriberPtr<LowState> lowstate_subscriber;
        lowstate_subscriber.reset(
            new unitree::robot::ChannelSubscriber<LowState>(kLowStateTopic));
        lowstate_subscriber->InitChannel(LowStateHandler, 1);

        // 主循环：每秒检查一次有没有新状态。
        // 这段是你最容易改的地方：
        // - 想 0.5 秒打印一次，就把 seconds(1) 改成 milliseconds(500)。
        // - 想降低刷屏，就改成 seconds(2) 或 seconds(3)。
        uint64_t last_seq = 0;
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            const ArmSnapshot snapshot = GetSnapshot();

            // 还没收到任何有效 lowstate，就提示等待。
            if (!snapshot.valid) {
                std::cout << "Waiting for " << kLowStateTopic
                          << " on " << network << " ...\n";
                continue;
            }

            // seq 变化说明收到了新帧，打印。
            // seq 没变说明 1 秒内没有新帧，提示网络或 DDS 可能有问题。
            if (snapshot.seq != last_seq) {
                PrintSnapshot(snapshot);
                last_seq = snapshot.seq;
            } else {
                std::cout << "No new lowstate frame received in the last second.\n";
            }
        }
    } catch (const std::exception& e) {
        // C++ 标准异常，比如 DDS 初始化失败、数组越界等，会走这里。
        std::cerr << "Program error: " << e.what() << "\n";
        return 1;
    } catch (...) {
        // 非标准异常走这里。一般不应该出现，但保留兜底提示。
        std::cerr << "Unknown program error.\n";
        return 1;
    }

    std::cout << "\nR1 arm state reader exited.\n";
    return 0;
}
