#include "../include/config.hpp"
#include <iostream>

namespace energy {

bool ConfigManager::load(const std::string& detector_yaml, const std::string& tracker_yaml,
                         const std::string& solvePnP_yaml) {
    try {
        // 加载检测器配置
        YAML::Node det_node = YAML::LoadFile(detector_yaml);
        detector_cfg_.color = det_node["color"].as<int>();
        auto red1 = det_node["red_range1"];
        detector_cfg_.red_h_low1 = red1["h_min"].as<int>();
        detector_cfg_.red_h_high1 = red1["h_max"].as<int>();
        detector_cfg_.red_s_min1 = red1["s_min"].as<int>();
        detector_cfg_.red_v_min1 = red1["v_min"].as<int>();
        auto red2 = det_node["red_range2"];
        detector_cfg_.red_h_low2 = red2["h_min"].as<int>();
        detector_cfg_.red_h_high2 = red2["h_max"].as<int>();
        detector_cfg_.red_s_min2 = red2["s_min"].as<int>();
        detector_cfg_.red_v_min2 = red2["v_min"].as<int>();
        auto blue = det_node["blue_range"];
        detector_cfg_.blue_h_low = blue["h_min"].as<int>();
        detector_cfg_.blue_h_high = blue["h_max"].as<int>();
        detector_cfg_.blue_s_min = blue["s_min"].as<int>();
        detector_cfg_.blue_v_min = blue["v_min"].as<int>();
        detector_cfg_.morph_size = det_node["morph_size"].as<int>();
        detector_cfg_.fanblade_min_area = det_node["fanblade_min_area"].as<double>();
        detector_cfg_.fanblade_max_area = det_node["fanblade_max_area"].as<double>();
        detector_cfg_.fanblade_ratio_min = det_node["fanblade_ratio_min"].as<double>();
        detector_cfg_.fanblade_ratio_max = det_node["fanblade_ratio_max"].as<double>();
        detector_cfg_.fanblade_circularity_max = det_node["fanblade_circularity_max"].as<double>();
        detector_cfg_.fanblade_solidity_min = det_node["fanblade_solidity_min"].as<double>();
        detector_cfg_.r_min_area = det_node["r_min_area"].as<double>();
        detector_cfg_.r_max_area = det_node["r_max_area"].as<double>();
        detector_cfg_.r_min_radius = det_node["r_min_radius"].as<double>();
        detector_cfg_.r_max_radius = det_node["r_max_radius"].as<double>();
        detector_cfg_.debug = det_node["debug"].as<int>();

        // 加载跟踪器配置
        YAML::Node tr_node = YAML::LoadFile(tracker_yaml);
        tracker_cfg_.center_smooth = tr_node["center_smooth"].as<double>();
        tracker_cfg_.radius_smooth = tr_node["radius_smooth"].as<double>();
        tracker_cfg_.angle_smooth = tr_node["angle_smooth"].as<double>();
        tracker_cfg_.blade_angle_smooth = tr_node["blade_angle_smooth"].as<double>();
        tracker_cfg_.angular_velocity_deg = tr_node["angular_velocity_deg"].as<double>();
        tracker_cfg_.max_lost_frames = tr_node["max_lost_frames"].as<int>();
        tracker_cfg_.match_distance_threshold = tr_node["match_distance_threshold"].as<double>();
        tracker_cfg_.num_blades = tr_node["num_blades"].as<int>();
        tracker_cfg_.jump_threshold_px = tr_node["jump_threshold_px"].as<double>();
        
        // 加载解算配置
        YAML::Node solve_node = YAML::LoadFile(solvePnP_yaml);
        solvePnP_cfg_.camera_matrix = solve_node["camera_matrix"].as<double>();
        solvePnP_cfg_.distortion_coefficients = solve_node["dist_coeffs"].as<double>();

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config: " << e.what() << std::endl;
        return false;
    }
}

} // namespace energy
