#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

namespace {

constexpr const char* kTopic = "rt/brainco/right/cmd";
constexpr float kSpeed = 1.0f;

using MotorCmds = unitree_go::msg::dds_::MotorCmds_;

std::atomic<bool> g_running{true};

void SignalHandler(int) {
    g_running.store(false);
}

void SetHandPose(MotorCmds& msg, const std::array<float, 6>& pose) {
    msg.cmds().resize(6);
    for (size_t i = 0; i < pose.size(); ++i) {
        msg.cmds()[i].q() = pose[i];
        msg.cmds()[i].dq() = kSpeed;
    }
}

void PrintPose(const char* label, const std::array<float, 6>& pose) {
    std::cout << label << " [thumb, thumb_aux, index, middle, ring, pinky] = [";
    for (size_t i = 0; i < pose.size(); ++i) {
        std::cout << pose[i];
        if (i + 1 < pose.size()) std::cout << ", ";
    }
    std::cout << "]\n";
}

void PublishPose(unitree::robot::ChannelPublisher<MotorCmds>& publisher,
                 MotorCmds& msg,
                 const std::array<float, 6>& pose,
                 int repeat_count,
                 std::chrono::milliseconds interval) {
    SetHandPose(msg, pose);
    for (int i = 0; i < repeat_count && g_running.load(); ++i) {
        publisher.Write(msg, 0);
        std::this_thread::sleep_for(interval);
    }
}

}  // namespace

int main(int argc, char** argv) {
    const std::string network = (argc > 1) ? argv[1] : "eth0";

    const std::array<float, 6> open_pose = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    const std::array<float, 6> thumb_up_pose = {0.f, 1.f, 1.f, 1.f, 1.f, 1.f};

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    std::cout << "BrainCo right hand thumb-up DDS test\n";
    std::cout << "Network interface: " << network << "\n";
    std::cout << "Topic: " << kTopic << "\n";
    PrintPose("Open pose", open_pose);
    PrintPose("Thumb-up pose", thumb_up_pose);

    unitree::robot::ChannelFactory::Instance()->Init(0, network.c_str());

    unitree::robot::ChannelPublisher<MotorCmds> publisher(kTopic);
    publisher.InitChannel();

    MotorCmds msg;
    msg.cmds().resize(6);

    std::cout << "Publishing open pose for 1 second...\n";
    PublishPose(publisher, msg, open_pose, 10, std::chrono::milliseconds(100));

    std::cout << "Publishing thumb-up pose continuously. Press Ctrl+C to reset and exit.\n";
    SetHandPose(msg, thumb_up_pose);
    while (g_running.load()) {
        publisher.Write(msg, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nCtrl+C received. Publishing open pose before exit...\n";
    g_running.store(true);
    PublishPose(publisher, msg, open_pose, 15, std::chrono::milliseconds(100));

    std::cout << "Done. The right hand should be reset to open pose.\n";
    return 0;
}
