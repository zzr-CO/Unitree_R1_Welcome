#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

namespace {

constexpr const char* kTopic = "rt/brainco/right/cmd";
constexpr float kSpeed = 1.0f;

using MotorCmds = unitree_go::msg::dds_::MotorCmds_;
using Pose = std::array<float, 6>;

const Pose kOpenPose = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
const Pose kScissorsPose = {1.f, 1.f, 0.f, 0.f, 1.f, 1.f};
const Pose kPaperPose = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
const Pose kRockPose = {1.f, 1.f, 1.f, 1.f, 1.f, 1.f};

std::atomic<bool> g_running{true};
std::mutex g_pose_mutex;
Pose g_current_pose = kOpenPose;

void SignalHandler(int) {
    g_running.store(false);
}

void SetHandPose(MotorCmds& msg, const Pose& pose) {
    msg.cmds().resize(6);
    for (size_t i = 0; i < pose.size(); ++i) {
        msg.cmds()[i].q() = pose[i];
        msg.cmds()[i].dq() = kSpeed;
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

void PublishPose(unitree::robot::ChannelPublisher<MotorCmds>& publisher,
                 MotorCmds& msg,
                 const Pose& pose,
                 int repeat_count,
                 std::chrono::milliseconds interval) {
    SetHandPose(msg, pose);
    for (int i = 0; i < repeat_count; ++i) {
        publisher.Write(msg, 0);
        std::this_thread::sleep_for(interval);
    }
}

void PublisherLoop(unitree::robot::ChannelPublisher<MotorCmds>* publisher) {
    MotorCmds msg;
    msg.cmds().resize(6);

    while (g_running.load()) {
        SetHandPose(msg, GetCurrentPose());
        publisher->Write(msg, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void PrintMenu() {
    std::cout << "\nInput gesture:\n"
              << "  1 = scissors\n"
              << "  2 = paper\n"
              << "  3 = rock\n"
              << "  Ctrl+C = reset and exit\n"
              << "> " << std::flush;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string network = (argc > 1) ? argv[1] : "eth0";

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    std::cout << "BrainCo right hand rock-paper-scissors DDS test\n";
    std::cout << "Network interface: " << network << "\n";
    std::cout << "Topic: " << kTopic << "\n";
    PrintPose("Scissors pose", kScissorsPose);
    PrintPose("Paper pose", kPaperPose);
    PrintPose("Rock pose", kRockPose);

    unitree::robot::ChannelFactory::Instance()->Init(0, network.c_str());

    unitree::robot::ChannelPublisher<MotorCmds> publisher(kTopic);
    publisher.InitChannel();

    MotorCmds msg;
    msg.cmds().resize(6);

    std::cout << "Publishing open pose for 1 second...\n";
    PublishPose(publisher, msg, kOpenPose, 10, std::chrono::milliseconds(100));

    SetCurrentPose(kOpenPose);
    std::thread publisher_thread(PublisherLoop, &publisher);

    while (g_running.load()) {
        PrintMenu();

        std::string input;
        if (!std::getline(std::cin, input)) {
            g_running.store(false);
            break;
        }

        if (input == "1") {
            SetCurrentPose(kScissorsPose);
            std::cout << "Gesture: scissors\n";
        } else if (input == "2") {
            SetCurrentPose(kPaperPose);
            std::cout << "Gesture: paper\n";
        } else if (input == "3") {
            SetCurrentPose(kRockPose);
            std::cout << "Gesture: rock\n";
        } else if (!input.empty()) {
            std::cout << "Invalid input. Use 1, 2, 3, or Ctrl+C.\n";
        }
    }

    if (publisher_thread.joinable()) {
        publisher_thread.join();
    }

    std::cout << "\nResetting right hand to open pose before exit...\n";
    PublishPose(publisher, msg, kOpenPose, 15, std::chrono::milliseconds(100));

    std::cout << "Done.\n";
    return 0;
}
