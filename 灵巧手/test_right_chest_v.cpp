#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/g1/arm/g1_arm_action_client.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace {

constexpr const char* kHandTopic = "rt/brainco/right/cmd";
constexpr float kFingerSpeed = 1.0f;
constexpr int32_t kArmAction = 25;     // wave_under_head
constexpr int32_t kReleaseArm = 99;    // release_arm

using MotorCmds = unitree_go::msg::dds_::MotorCmds_;
using Pose = std::array<float, 6>;

const Pose kOpenPose = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
const Pose kVictoryPose = {1.f, 1.f, 0.f, 0.f, 1.f, 1.f};

std::atomic<bool> g_exit_requested{false};
std::atomic<bool> g_publish_running{true};
std::mutex g_pose_mutex;
Pose g_current_pose = kOpenPose;

void SignalHandler(int) {
    g_exit_requested.store(true);
}

void SetHandPose(MotorCmds& msg, const Pose& pose) {
    msg.cmds().resize(6);
    for (size_t i = 0; i < pose.size(); ++i) {
        msg.cmds()[i].q() = pose[i];
        msg.cmds()[i].dq() = kFingerSpeed;
    }
}

void PrintPose(const char* label, const Pose& pose) {
    std::cout << label << " [thumb, thumb_aux, index, middle, ring, pinky] = [";
    for (size_t i = 0; i < pose.size(); ++i) {
        std::cout << pose[i];
        if (i + 1 < pose.size()) std::cout << ", ";
    }
    std::cout << "]\n";
}

void SetCurrentPose(const Pose& pose) {
    std::lock_guard<std::mutex> lock(g_pose_mutex);
    g_current_pose = pose;
}

Pose GetCurrentPose() {
    std::lock_guard<std::mutex> lock(g_pose_mutex);
    return g_current_pose;
}

void PublishLoop(unitree::robot::ChannelPublisher<MotorCmds>& publisher) {
    MotorCmds msg;
    msg.cmds().resize(6);
    while (g_publish_running.load()) {
        SetHandPose(msg, GetCurrentPose());
        publisher.Write(msg, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void HoldPoseFor(const Pose& pose, std::chrono::milliseconds duration) {
    SetCurrentPose(pose);
    std::this_thread::sleep_for(duration);
}

}  // namespace

int main(int argc, char** argv) {
    const std::string network = (argc > 1) ? argv[1] : "eth10";

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    std::cout << "R1 right hand chest V-pose test\n";
    std::cout << "Network interface: " << network << "\n";
    std::cout << "Hand topic: " << kHandTopic << "\n";
    std::cout << "Arm action: " << kArmAction << " (wave_under_head)\n";
    PrintPose("Open pose", kOpenPose);
    PrintPose("V pose", kVictoryPose);

    unitree::robot::ChannelFactory::Instance()->Init(0, network.c_str());

    unitree::robot::ChannelPublisher<MotorCmds> hand_publisher(kHandTopic);
    hand_publisher.InitChannel();

    auto arm = std::make_shared<unitree::robot::g1::G1ArmActionClient>();
    bool arm_ok = false;
    try {
        arm->Init();
        arm->SetTimeout(10.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        arm_ok = true;
        std::cout << "Arm action client ready.\n";
    } catch (...) {
        std::cerr << "Failed to initialize arm action client.\n";
        return 1;
    }

    std::thread publisher_thread(PublishLoop, std::ref(hand_publisher));

    std::cout << "Publishing open hand pose for 1 second...\n";
    HoldPoseFor(kOpenPose, std::chrono::seconds(1));

    std::cout << "Executing arm action 25 (wave_under_head)...\n";
    SetCurrentPose(kVictoryPose);
    int ret = arm->ExecuteAction(kArmAction);
    if (ret != 0) {
        std::cerr << "Arm action failed, ret=" << ret << "\n";
    }

    std::cout << "Publishing V pose continuously. Press Ctrl+C to reset and exit.\n";
    while (!g_exit_requested.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nOpening hand before arm release...\n";
    HoldPoseFor(kOpenPose, std::chrono::milliseconds(1500));

    if (arm_ok) {
        std::cout << "Releasing arm...\n";
        ret = arm->ExecuteAction(kReleaseArm);
        if (ret != 0) {
            std::cerr << "Release arm failed, ret=" << ret << "\n";
        }
    }

    g_publish_running.store(false);
    if (publisher_thread.joinable()) {
        publisher_thread.join();
    }

    std::cout << "Done.\n";
    return 0;
}
