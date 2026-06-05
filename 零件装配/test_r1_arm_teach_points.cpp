/*
 * R1 right arm teach point recorder.
 *
 * This is the second low-level motion development test for this project.
 * It only subscribes to rt/lowstate and records right-arm joint angles when
 * you press a key. It does not publish rt/lowcmd and does not control motors.
 *
 * Compile on R1 from ~/unitree_sdk2-1.0:
 *   g++ -std=c++17 test_r1_arm_teach_points.cpp \
 *       -I./include -I./thirdparty/include/ddscxx \
 *       -L./lib/aarch64 -L./thirdparty/lib/aarch64 \
 *       -lunitree_sdk2 -lddscxx -lddsc -lpthread \
 *       -Wl,-rpath,./lib/aarch64:./thirdparty/lib/aarch64 \
 *       -o test_r1_arm_teach_points
 *
 * Run:
 *   ./test_r1_arm_teach_points eth10
 *
 * 学习目标：
 *   1. 学会“示教”的基本思想：手动摆姿态，程序记录关节角。
 *   2. 学会把一组关节角保存成一个“点位”，后续可用于轨迹复现。
 *   3. 学会用 C++ 数组、结构体、回调函数、互斥锁和终端输入组织小工具。
 *
 * 你可以改：
 *   - 默认网口 eth10。
 *   - kTeachPoints 里的点位名称和中文说明。
 *   - PrintCurrentRightArm() 里显示的小数位数。
 *
 * 暂时不要改：
 *   - kLowStateTopic = "rt/lowstate"。
 *   - 右臂关节 ID：22-26。
 *   - Crc32Core() 和 LowStateHandler() 的 CRC 校验逻辑。
 *
 * 安全说明：
 *   本程序没有 ChannelPublisher，没有 Write() 电机命令，不发布 rt/lowcmd。
 *   它只读 R1 当前右臂状态，不会主动让机器人运动。
 */

// ===== 1. C++ 标准库头文件 =====
// 你应该学会：
// - array：保存固定数量的右臂 5 个关节。
// - atomic：保存 Ctrl+C 退出开关。
// - mutex：保护 DDS 回调线程和主线程共享的右臂状态。
// - iomanip：控制终端表格宽度和小数位数。
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
// LowState_ 是 R1 底层状态消息类型。
// ChannelSubscriber 只负责订阅消息，不会发送控制命令。
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

// ===== 3. 类型别名 =====
// 你应该学会：using 可以给很长的类型起一个短名字。
using LowState = unitree_hg::msg::dds_::LowState_;
using ArmPose = std::array<float, 5>;

// ===== 4. 全局常量 =====
// 你可以改：kDefaultNetwork，如果你的 DDS 网口不是 eth10。
// 暂时不要改：kLowStateTopic，它是 R1 底层状态 DDS 话题。
static constexpr const char* kLowStateTopic = "rt/lowstate";
static constexpr const char* kDefaultNetwork = "eth10";
static std::atomic<bool> g_running{true};

// ===== 5. 右臂关节信息 =====
// 这里记录右臂 5 个关节在 LowState 里的 IDL 编号。
// 暂时不要改 idl_index，后续底层控制也会用同一套编号。
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

// ===== 6. 点位定义 =====
// TeachPoint 是“示教点位”的定义。
// - key：你在终端输入的按键。
// - name：英文点位名，后续可直接用于 C++ 代码。
// - chinese_name：中文说明，方便现场操作。
// - description：这个点位在装配流程中的意义。
// 你可以改：点位名称和说明。
struct TeachPoint {
    char key;
    const char* name;
    const char* chinese_name;
    const char* description;
    bool recorded = false;
    ArmPose q{};
};

static std::array<TeachPoint, 7> kTeachPoints{{
    {'1', "HOME",           "安全初始位",   "手臂处于安全、自然、可开始任务的位置"},
    {'2', "PART_ABOVE",     "零件上方",     "右手移动到待抓零件的正上方"},
    {'3', "PART_GRASP",     "抓取位置",     "右手靠近零件，灵巧手适合闭合抓取的位置"},
    {'4', "PART_LIFT",      "抓取抬高位",   "抓住零件后，向上抬起避免碰撞桌面的位置"},
    {'5', "ASSEMBLY_ABOVE", "装配位上方",   "移动到装配目标上方，准备下探装配"},
    {'6', "ASSEMBLY_INSERT","装配插入位",   "对准并模拟插入/放置零件的位置"},
    {'7', "RETURN_HOME",    "返回安全位",   "装配完成后退出工作区的位置"},
}};

// ===== 7. 最新右臂状态缓存 =====
// DDS 回调线程会不断更新 g_latest_right_arm。
// 主线程在你输入 1-7 时，把当前缓存保存为某个点位。
static std::mutex g_pose_mutex;
static ArmPose g_latest_right_arm{};
static bool g_pose_valid = false;
static uint64_t g_pose_seq = 0;

// ===== 8. Ctrl+C 信号处理 =====
// 按 Ctrl+C 后，程序不会突然崩掉，而是让主循环自然退出。
static void SignalHandler(int) {
    g_running = false;
}

// ===== 9. CRC 校验函数 =====
// CRC 是底层消息完整性校验，可以理解成“数据封条”。
// 暂时不要改：这个函数来自 Unitree 底层示例的常见写法。
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

// ===== 10. 弧度转角度 =====
// R1 底层 q 使用 rad，现场观察时 deg 更直观。
static float RadToDeg(float rad) {
    return rad * 180.0f / 3.14159265358979323846f;
}

// ===== 11. DDS 状态回调函数 =====
// 每收到一帧 rt/lowstate，这个函数就会被 SDK 自动调用。
// 它只做两件事：
// 1. 校验消息是否完整。
// 2. 读取右臂 5 个关节的 q(rad)，保存到 g_latest_right_arm。
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
    g_latest_right_arm = pose;
    g_pose_valid = true;
    ++g_pose_seq;
}

// ===== 12. 读取当前右臂姿态 =====
// 这里返回一份拷贝，避免主线程直接读取正在被回调线程修改的数据。
static bool GetLatestRightArm(ArmPose& pose, uint64_t& seq) {
    std::lock_guard<std::mutex> lock(g_pose_mutex);
    if (!g_pose_valid) return false;
    pose = g_latest_right_arm;
    seq = g_pose_seq;
    return true;
}

// ===== 13. 打印当前右臂角度 =====
// 你可以改：setprecision(3)，比如改成 4 就会显示更多小数。
// 你应该学会：同一个 q 同时用 rad 和 deg 显示，rad 给程序用，deg 给人看。
static void PrintCurrentRightArm(const ArmPose& pose, uint64_t seq) {
    std::cout << "\n当前右臂姿态 seq=" << seq << "\n";
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

// ===== 14. 打印操作菜单 =====
// 你应该学会：交互式测试程序通常用简单菜单降低误操作。
static void PrintMenu() {
    std::cout << "\n================ 示教点位记录器 ================\n";
    std::cout << "把右臂手动摆到目标位置后，输入对应编号保存当前关节角。\n";
    for (const auto& point : kTeachPoints) {
        std::cout << "  " << point.key << " = " << point.chinese_name
                  << " (" << point.name << ")"
                  << (point.recorded ? " [已记录]" : " [未记录]") << "\n";
    }
    std::cout << "  v = 查看当前右臂角度\n";
    std::cout << "  p = 打印已记录的 C++ 点位表\n";
    std::cout << "  h = 显示本菜单\n";
    std::cout << "  q = 退出\n";
    std::cout << "  Ctrl+C = 退出\n";
    std::cout << "> " << std::flush;
}

// ===== 15. 根据按键查找点位 =====
// 输入 '1' 到 '7' 时，用这个函数找到对应 TeachPoint。
static TeachPoint* FindTeachPoint(char key) {
    for (auto& point : kTeachPoints) {
        if (point.key == key) return &point;
    }
    return nullptr;
}

// ===== 16. 保存当前姿态到点位 =====
// 这个函数是真正的“示教记录”动作：
// 当前右臂姿态 -> 保存到某个 TeachPoint。
static void RecordTeachPoint(TeachPoint& point) {
    ArmPose pose{};
    uint64_t seq = 0;
    if (!GetLatestRightArm(pose, seq)) {
        std::cout << "[WARN] 还没有收到 rt/lowstate，无法记录点位。\n";
        return;
    }

    point.q = pose;
    point.recorded = true;

    std::cout << "\n已记录 " << point.chinese_name
              << " (" << point.name << "), seq=" << seq << "\n";
    PrintCurrentRightArm(point.q, seq);
}

// ===== 17. 打印 C++ 点位表 =====
// 这个输出是给后续“自动复现动作”的程序复制使用的。
// 没记录的点位会用注释标出，不会假装有数据。
static void PrintCppPoseTable() {
    std::cout << "\n// 右臂示教点位表\n";
    std::cout << "// 关节顺序: [R_SHOULDER_PITCH, R_SHOULDER_ROLL, R_SHOULDER_YAW, R_ELBOW, R_WRIST_ROLL]\n";
    std::cout << "struct RightArmPose {\n";
    std::cout << "    const char* name;\n";
    std::cout << "    std::array<float, 5> q;\n";
    std::cout << "};\n\n";
    std::cout << "static const RightArmPose kRightArmTeachPoints[] = {\n";

    std::cout << std::fixed << std::setprecision(6);
    for (const auto& point : kTeachPoints) {
        if (!point.recorded) {
            std::cout << "    // " << point.name << " 未记录: "
                      << point.chinese_name << "\n";
            continue;
        }

        std::cout << "    {\"" << point.name << "\", {";
        for (size_t i = 0; i < point.q.size(); ++i) {
            std::cout << point.q.at(i) << "f";
            if (i + 1 < point.q.size()) std::cout << ", ";
        }
        std::cout << "}}, // " << point.chinese_name << "\n";
    }
    std::cout << "};\n";
    std::cout << std::defaultfloat;
}

// ===== 18. 主函数入口 =====
// 整体流程：
// 1. 初始化 DDS。
// 2. 订阅 rt/lowstate。
// 3. 等待用户输入。
// 4. 输入 1-7 保存当前右臂点位。
// 5. 输入 p 打印 C++ 点位表。
int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    // 你可以改：默认网口 kDefaultNetwork。
    // 更推荐运行时指定：./test_r1_arm_teach_points eth10
    const std::string network = (argc >= 2) ? argv[1] : kDefaultNetwork;

    std::cout << "R1 right arm teach point recorder\n";
    std::cout << "Network interface: " << network << "\n";
    std::cout << "Subscribe topic: " << kLowStateTopic << "\n";
    std::cout << "Safety: read-only, no ChannelPublisher, no Write(), no rt/lowcmd.\n";

    try {
        // 初始化 Unitree DDS 通信。
        unitree::robot::ChannelFactory::Instance()->Init(0, network);

        // 创建 rt/lowstate 订阅器。
        unitree::robot::ChannelSubscriberPtr<LowState> lowstate_subscriber;
        lowstate_subscriber.reset(
            new unitree::robot::ChannelSubscriber<LowState>(kLowStateTopic));
        lowstate_subscriber->InitChannel(LowStateHandler, 1);

        std::cout << "Waiting for first lowstate frame...\n";
        while (g_running && !g_pose_valid) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (!g_running) {
            std::cout << "\nExit before first lowstate frame.\n";
            return 0;
        }

        PrintMenu();

        std::string input;
        while (g_running && std::getline(std::cin, input)) {
            if (input.empty()) {
                PrintMenu();
                continue;
            }

            const char key = input.at(0);
            if (TeachPoint* point = FindTeachPoint(key)) {
                RecordTeachPoint(*point);
            } else if (key == 'v' || key == 'V') {
                ArmPose pose{};
                uint64_t seq = 0;
                if (GetLatestRightArm(pose, seq)) {
                    PrintCurrentRightArm(pose, seq);
                } else {
                    std::cout << "[WARN] 还没有收到 rt/lowstate。\n";
                }
            } else if (key == 'p' || key == 'P') {
                PrintCppPoseTable();
            } else if (key == 'h' || key == 'H') {
                PrintMenu();
            } else if (key == 'q' || key == 'Q') {
                break;
            } else {
                std::cout << "无效输入。输入 h 查看菜单。\n";
            }

            if (g_running) {
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

    std::cout << "\nR1 right arm teach point recorder exited.\n";
    return 0;
}
