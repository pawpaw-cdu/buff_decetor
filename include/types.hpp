#ifndef ENERGY_DETECTOR_TYPES_HPP
#define ENERGY_DETECTOR_TYPES_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace energy {

enum class BladeStatus { ACTIVE, NOACTIVE, NOTHING };

struct Blade {
    std::vector<cv::Point2f> centers;
    std::vector<BladeStatus> status;
};

struct EnergyBlade {
    cv::Point2f center;
    int id;
    float angle_rad;
    cv::Point2f object_points[4];
    cv::Point2f transforme_points[4];
};

struct DetectorResult {
    std::vector<cv::Point2f> blade_centers;
    cv::Point2f r_center;
    float r_radius;
    bool r_found;
};

struct TrackerResult {
    std::vector<EnergyBlade> blades;
    cv::Point2f model_center;
    float model_radius;
    bool valid;
};

} // namespace energy

#endif
