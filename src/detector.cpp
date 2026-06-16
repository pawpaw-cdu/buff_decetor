#include "../include/detector.hpp"
#include <iostream>

namespace energy {

Detector::Detector(const DetectorConfig& cfg) : cfg_(cfg) {}

cv::Mat Detector::preprocessMask(const cv::Mat& bgr, int color) {
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    if (color == 1) {
        // 红色
        cv::Mat mask1, mask2;
        cv::inRange(hsv, cv::Scalar(cfg_.red_h_low1, cfg_.red_s_min1, cfg_.red_v_min1),
                    cv::Scalar(cfg_.red_h_high1, 255, 255), mask1);
        //cv::inRange(hsv, cv::Scalar(cfg_.red_h_low2, cfg_.red_s_min2, cfg_.red_v_min2),
        //            cv::Scalar(cfg_.red_h_high2, 255, 255), mask2);
        //mask = mask1 | mask2;
        mask = mask1;
    } else {
        cv::inRange(hsv, cv::Scalar(cfg_.blue_h_low, cfg_.blue_s_min, cfg_.blue_v_min),
                    cv::Scalar(cfg_.blue_h_high, 255, 255), mask);
    }
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cfg_.morph_size, cfg_.morph_size));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    if (cfg_.debug) {
        cv::imshow("mask_blade", mask);
    }
    return mask;
}

std::vector<cv::Point2f> Detector::extractBladeCenters(const cv::Mat& mask) {
    std::vector<cv::Point2f> centers;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area < cfg_.fanblade_min_area || area > cfg_.fanblade_max_area) continue;
        cv::RotatedRect rect = cv::minAreaRect(cnt);
        float w = rect.size.width, h = rect.size.height;
        float ratio = std::min(w, h) / std::max(w, h);
        if (ratio < cfg_.fanblade_ratio_min || ratio > cfg_.fanblade_ratio_max) continue;
        double perimeter = cv::arcLength(cnt, true);
        double circularity = 4 * CV_PI * area / (perimeter * perimeter);
        if (circularity > cfg_.fanblade_circularity_max) continue;
        std::vector<cv::Point> hull;
        cv::convexHull(cnt, hull);
        double hull_area = cv::contourArea(hull);
        double solidity = area / hull_area;
        if (solidity < cfg_.fanblade_solidity_min) continue;
        cv::Moments m = cv::moments(cnt);
        if (m.m00 > 0) {
            centers.push_back(cv::Point2f(m.m10 / m.m00, m.m01 / m.m00));
        }
    }
    return centers;
}

bool Detector::detectRCenter(const cv::Mat& bgr, cv::Point2f& center, float& radius) {
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    if (cfg_.color == 1) {
        cv::Mat mask1, mask2;
        cv::inRange(hsv, cv::Scalar(cfg_.red_h_low1, cfg_.red_s_min1, cfg_.red_v_min1),
                    cv::Scalar(cfg_.red_h_high1, 255, 255), mask1);
        //cv::inRange(hsv, cv::Scalar(cfg_.red_h_low2, cfg_.red_s_min2, cfg_.red_v_min2),
        //            cv::Scalar(cfg_.red_h_high2, 255, 255), mask2);
        mask = mask1;
    } else {
        cv::inRange(hsv, cv::Scalar(cfg_.blue_h_low, cfg_.blue_s_min, cfg_.blue_v_min),
                    cv::Scalar(cfg_.blue_h_high, 255, 255), mask);
    }
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area < cfg_.r_min_area || area > cfg_.r_max_area) continue;
        cv::RotatedRect rect = cv::minAreaRect(cnt);
        float w = rect.size.width, h = rect.size.height;
        float ratio = std::min(w, h) / std::max(w, h);
        if (ratio < 0.7) continue;
        double perimeter = cv::arcLength(cnt, true);
        double circularity = 4 * CV_PI * area / (perimeter * perimeter);
        if (circularity < 0.3) continue;
        cv::Point2f cnt_center;
        float cnt_radius;
        cv::minEnclosingCircle(cnt, cnt_center, cnt_radius);
        if (cnt_radius >= cfg_.r_min_radius && cnt_radius <= cfg_.r_max_radius) {
            center = cnt_center;
            radius = 1.2*cnt_radius;
            return true;
        }
    }
    return false;
}

DetectorResult Detector::process(const cv::Mat& bgr) {
    DetectorResult result;
    result.r_found = false;
    cv::Mat mask = preprocessMask(bgr, cfg_.color);
    result.blade_centers = extractBladeCenters(mask);
    result.r_found = detectRCenter(bgr, result.r_center, result.r_radius);
    return result;
}

} // namespace energy
