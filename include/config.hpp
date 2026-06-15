#ifndef ENERGY_DETECTOR_CONFIG_HPP
#define ENERGY_DETECTOR_CONFIG_HPP

#include <string>
#include <yaml-cpp/yaml.h>

namespace energy {

struct DetectorConfig {
    int color;                   // 1:red, 0:blue
    // HSV ranges
    int red_h_low1, red_h_high1, red_s_min1, red_v_min1;
    int red_h_low2, red_h_high2, red_s_min2, red_v_min2;
    int blue_h_low, blue_h_high, blue_s_min, blue_v_min;
    int morph_size;
    double fanblade_min_area, fanblade_max_area;
    double fanblade_ratio_min, fanblade_ratio_max;
    double fanblade_circularity_max;
    double fanblade_solidity_min;
    double r_min_area, r_max_area, r_min_radius, r_max_radius;
    int debug;
};

struct TrackerConfig {
    double center_smooth;
    double radius_smooth;
    double angle_smooth;
    double blade_angle_smooth;
    double angular_velocity_deg;
    int max_lost_frames;
    double match_distance_threshold;
    int num_blades;
    double jump_threshold_px;
};

class ConfigManager {
public:
    bool load(const std::string& detector_yaml, const std::string& tracker_yaml);
    const DetectorConfig& getDetector() const { return detector_cfg_; }
    const TrackerConfig& getTracker() const { return tracker_cfg_; }

private:
    DetectorConfig detector_cfg_;
    TrackerConfig tracker_cfg_;
};

} // namespace energy

#endif

