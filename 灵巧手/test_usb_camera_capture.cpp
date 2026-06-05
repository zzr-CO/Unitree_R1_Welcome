#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

namespace {

bool IsInteger(const std::string& text) {
    if (text.empty()) return false;
    size_t start = (text[0] == '-' || text[0] == '+') ? 1 : 0;
    if (start == text.size()) return false;
    for (size_t i = start; i < text.size(); ++i) {
        if (text[i] < '0' || text[i] > '9') return false;
    }
    return true;
}

void PrintUsage(const char* program) {
    std::cout
        << "USB camera capture test\n"
        << "Usage:\n"
        << "  " << program << " [device] [output.jpg]\n\n"
        << "Examples:\n"
        << "  " << program << " 0 hand_camera.jpg\n"
        << "  " << program << " /dev/video0 hand_camera.jpg\n\n"
        << "Notes:\n"
        << "  device can be a camera index such as 0 or a V4L2 path such as /dev/video0.\n"
        << "  This program only reads the camera and saves one frame. It does not control the hand.\n";
}

void PrintCameraInfo(cv::VideoCapture& camera) {
    std::cout << "Camera opened.\n";
    std::cout << "Width: " << camera.get(cv::CAP_PROP_FRAME_WIDTH) << "\n";
    std::cout << "Height: " << camera.get(cv::CAP_PROP_FRAME_HEIGHT) << "\n";
    std::cout << "FPS: " << camera.get(cv::CAP_PROP_FPS) << "\n";
    std::cout << "FourCC: " << static_cast<int>(camera.get(cv::CAP_PROP_FOURCC)) << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1) {
        const std::string first_arg = argv[1];
        if (first_arg == "-h" || first_arg == "--help") {
            PrintUsage(argv[0]);
            return 0;
        }
    }

    const std::string device = (argc > 1) ? argv[1] : "0";
    const std::string output_path = (argc > 2) ? argv[2] : "hand_camera.jpg";

    std::cout << "Opening camera device: " << device << "\n";

    cv::VideoCapture camera;
    if (IsInteger(device)) {
        camera.open(std::stoi(device), cv::CAP_V4L2);
    } else {
        camera.open(device, cv::CAP_V4L2);
    }

    if (!camera.isOpened()) {
        std::cerr << "Failed to open camera: " << device << "\n";
        std::cerr << "Try checking devices with: ls -l /dev/video* && v4l2-ctl --list-devices\n";
        return 1;
    }

    camera.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    camera.set(cv::CAP_PROP_FPS, 30);

    PrintCameraInfo(camera);

    cv::Mat frame;
    for (int i = 0; i < 20; ++i) {
        camera >> frame;
        if (!frame.empty()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (frame.empty()) {
        std::cerr << "Camera opened, but no frame was received.\n";
        return 2;
    }

    if (!cv::imwrite(output_path, frame)) {
        std::cerr << "Failed to save image: " << output_path << "\n";
        return 3;
    }

    std::cout << "Saved frame: " << output_path << "\n";
    return 0;
}
