#ifndef ENERGY_DETECTOR_DETECTOR_HPP
#define ENERGY_DETECTOR_DETECTOR_HPP

#include "types.hpp"
#include "config.hpp"

namespace energy {

class Detector {
public:
    explicit Detector(const DetectorConfig& cfg);
    DetectorResult process(const cv::Mat& bgr);

private:
    DetectorConfig cfg_;
    cv::Mat preprocessMask(const cv::Mat& bgr, int color);
    std::vector<cv::Point2f> extractBladeCenters(const cv::Mat& mask);
    bool detectRCenter(const cv::Mat& bgr, cv::Point2f& center, float& radius);
};

} // namespace energy

#endif
