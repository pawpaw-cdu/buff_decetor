#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "../include/config.hpp"
#include "../include/detector.hpp"
#include "../include/tracker.hpp"
#include "../include/utils.hpp"

int main() {
    std::string video_path = "../buff_red.mp4";
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open video file " << video_path << std::endl;
        return -1;
    }

    energy::ConfigManager cfg_mgr;
    if (!cfg_mgr.load("../config/detector.yaml", "../config/tracker.yaml")) {
        std::cerr << "Failed to load config" << std::endl;
        return -1;
    }

    energy::Detector detector(cfg_mgr.getDetector());
    energy::Tracker tracker(cfg_mgr.getTracker());

    double last_timestamp = 0;
    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        double dt = (last_timestamp > 0) ? (now - last_timestamp) : 0.033;
        if (dt > 0.1) dt = 0.033;
        last_timestamp = now;

        auto det_result = detector.process(frame);
        auto track_result = tracker.update(det_result, dt);

        cv::Mat display = frame.clone();
        energy::drawModel(display, track_result);
        energy::showBladesROI(frame, det_result.blade_centers, 80);

        cv::resize(display, display, cv::Size(960, 540));
        cv::imshow("Energy Detection", display);

        if (cv::waitKey(10) == 'q') break;
    }
    return 0;
}
