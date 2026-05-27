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
#include <vector>

namespace {

constexpr const char* kTopic = "rt/brainco/left/cmd";
constexpr float kSpeed = 1.0f;

using MotorCmds = unitree_go::msg::dds_::MotorCmds_;
using Pose = std::array<float, 6>;

struct Gesture {
    const char* key;
    const char* english_name;
    const char* chinese_name;
    Pose pose;
};

const Pose kOpenPose = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

const std::vector<Gesture> kGestures = {
    {"0", "open",      "复位张开",      {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}},
    {"1", "half-fist", "半握拳",        {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f}},
    {"2", "v-sign",    "剪刀手/比耶",   {1.f, 1.f, 0.f, 0.f, 1.f, 1.f}},
    {"3", "thumb-up",  "大拇指点赞",    {0.f, 0.f, 1.f, 1.f, 1.f, 1.f}},
    {"4", "point",     "竖起食指",      {1.f, 1.f, 0.f, 1.f, 1.f, 1.f}},
    {"5", "shaka",     "电话手势/六",   {0.f, 0.f, 1.f, 1.f, 1.f, 0.f}},
    {"6", "relaxed",   "轻松自然手",    {0.2f, 0.2f, 0.25f, 0.25f, 0.25f, 0.25f}},
    {"7", "soft-fist", "轻握拳",        {0.75f, 0.75f, 0.75f, 0.75f, 0.75f, 0.75f}},
    {"8", "ok",       "OK手势",        {0.7f, 0.7f, 0.65f, 0.f, 0.f, 0.f}},
    {"9", "three",     "三指展示",      {1.f, 1.f, 0.f, 0.f, 0.f, 1.f}},
    {"a", "four",      "四指展示",      {1.f, 1.f, 0.f, 0.f, 0.f, 0.f}},
    {"c", "cup-grip",  "抓取杯子",      {0.45f, 0.45f, 0.55f, 0.55f, 0.55f, 0.55f}},
    {"d", "pinch-soft", "轻捏候选",     {0.65f, 0.65f, 0.65f, 0.2f, 0.2f, 0.2f}},
    {"e", "pinch-strong", "强捏候选",   {0.85f, 0.85f, 0.85f, 0.35f, 0.35f, 0.35f}},
};

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

void RunFingerWave() {
    const Pose stages[] = {
        {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
        {0.2f, 0.2f, 0.f, 0.f, 0.f, 0.f},
        {0.4f, 0.4f, 0.2f, 0.f, 0.f, 0.f},
        {0.6f, 0.6f, 0.4f, 0.2f, 0.f, 0.f},
        {0.8f, 0.8f, 0.6f, 0.4f, 0.2f, 0.f},
        {1.f, 1.f, 0.8f, 0.6f, 0.4f, 0.2f},
        {0.8f, 0.8f, 1.f, 0.8f, 0.6f, 0.4f},
        {0.6f, 0.6f, 0.8f, 1.f, 0.8f, 0.6f},
        {0.4f, 0.4f, 0.6f, 0.8f, 1.f, 0.8f},
        {0.2f, 0.2f, 0.4f, 0.6f, 0.8f, 1.f},
        {0.f, 0.f, 0.2f, 0.4f, 0.6f, 0.8f},
        {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
    };

    std::cout << "Animation: finger-wave\n";
    for (const auto& stage : stages) {
        if (!g_running.load()) break;
        SetCurrentPose(stage);
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
}

void RunBreathingCurl() {
    const Pose stages[] = {
        {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
        {0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f},
        {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f},
        {0.75f, 0.75f, 0.75f, 0.75f, 0.75f, 0.75f},
        {1.f, 1.f, 1.f, 1.f, 1.f, 1.f},
        {0.75f, 0.75f, 0.75f, 0.75f, 0.75f, 0.75f},
        {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f},
        {0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f},
        {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
    };

    std::cout << "Animation: breathing-curl\n";
    for (int loop = 0; loop < 2 && g_running.load(); ++loop) {
        for (const auto& stage : stages) {
            if (!g_running.load()) break;
            SetCurrentPose(stage);
            std::this_thread::sleep_for(std::chrono::milliseconds(220));
        }
    }
}

void PrintMenu() {
    std::cout << "\n左手灵巧手动作表\n";
    for (const auto& gesture : kGestures) {
        std::cout << "  " << gesture.key << " = " << gesture.chinese_name
                  << " (" << gesture.english_name << ")\n";
    }
    std::cout << "  w = 波浪滚动演示 (finger-wave, 暂不作为正式动作)\n"
              << "  b = 呼吸握拳演示 (breathing-curl, 暂不作为正式动作)\n"
              << "  Ctrl+C = reset and exit\n"
              << "> " << std::flush;
}

const Gesture* FindGesture(const std::string& key) {
    for (const auto& gesture : kGestures) {
        if (key == gesture.key) return &gesture;
    }
    return nullptr;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string network = (argc > 1) ? argv[1] : "eth0";

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    std::cout << "BrainCo left hand gesture lab DDS test\n";
    std::cout << "Network interface: " << network << "\n";
    std::cout << "Topic: " << kTopic << "\n";
    std::cout << "Finger value: 0.0=open, 0.5=half curl, 1.0=full curl\n";
    std::cout << "Finger order: [thumb, thumb_aux, index, middle, ring, pinky]\n";

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

        if (const Gesture* gesture = FindGesture(input)) {
            SetCurrentPose(gesture->pose);
            std::cout << "Gesture: " << gesture->chinese_name
                      << " (" << gesture->english_name << ")\n";
        } else if (input == "w") {
            RunFingerWave();
        } else if (input == "b") {
            RunBreathingCurl();
        } else if (!input.empty()) {
            std::cout << "Invalid input.\n";
        }
    }

    if (publisher_thread.joinable()) {
        publisher_thread.join();
    }

    std::cout << "\nResetting left hand to open pose before exit...\n";
    PublishPose(publisher, msg, kOpenPose, 15, std::chrono::milliseconds(100));

    std::cout << "Done.\n";
    return 0;
}
