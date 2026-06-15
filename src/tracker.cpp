#include "../include/tracker.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace energy {

Tracker::Tracker(const TrackerConfig& cfg) : cfg_(cfg), first_frame_(true), last_timestamp_(0.0),
                                              last_ref_angle_(0.0), has_ref_angle_(false), predicted_ref_angle_(0.0) {
    reset();
}

void Tracker::reset() {
    trackers_.clear();
    for (int i = 0; i < cfg_.num_blades; ++i) {
        BladeTracker tr;
        tr.id = i;
        tr.lost = true;
        tr.lost_count = 0;
        tr.center = cv::Point2f(0,0);
        tr.angle = 0;
        trackers_.push_back(tr);
    }
    smoothed_angles_.assign(cfg_.num_blades, 0.0f);
    angle_initialized_.assign(cfg_.num_blades, false);
    smoothed_center_ = cv::Point2f(0,0);
    smoothed_radius_ = 0;
    first_frame_ = true;
    last_timestamp_ = 0;
    has_ref_angle_ = false;
}

std::vector<int> Tracker::hungarianMatch(const std::vector<cv::Point2f>& detected) {
    int n = detected.size();
    int m = cfg_.num_blades;
    std::vector<std::vector<float>> cost(n, std::vector<float>(m, 0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            float dist = cv::norm(detected[i] - trackers_[j].center);
            cost[i][j] = dist;
            if (trackers_[j].lost) cost[i][j] += cfg_.match_distance_threshold;
        }
    }
    // 贪心匹配
    std::vector<int> assignment(m, -1);
    std::vector<bool> used(n, false);
    for (int iter = 0; iter < m; ++iter) {
        int best_i = -1, best_j = -1;
        float best_cost = 1e9;
        for (int j = 0; j < m; ++j) {
            if (assignment[j] != -1) continue;
            for (int i = 0; i < n; ++i) {
                if (used[i]) continue;
                if (cost[i][j] < best_cost) {
                    best_cost = cost[i][j];
                    best_i = i;
                    best_j = j;
                }
            }
        }
        if (best_i != -1 && best_cost < cfg_.match_distance_threshold * 2) {
            assignment[best_j] = best_i;
            used[best_i] = true;
        }
    }
    return assignment;
}

void Tracker::updateTrackers(const std::vector<cv::Point2f>& detected, double dt) {
    if (detected.empty()) {
        for (int j = 0; j < cfg_.num_blades; ++j) {
            trackers_[j].lost = true;
            trackers_[j].lost_count++;
        }
        return;
    }
    std::vector<int> assign = hungarianMatch(detected);
    for (int j = 0; j < cfg_.num_blades; ++j) {
        int matched_idx = assign[j];
        if (matched_idx != -1) {
            trackers_[j].center = detected[matched_idx];
            trackers_[j].lost = false;
            trackers_[j].lost_count = 0;
            float dx = trackers_[j].center.x - smoothed_center_.x;
            float dy = trackers_[j].center.y - smoothed_center_.y;
            float measured_angle = atan2(dy, dx);
            if (angle_initialized_[j]) {
                float diff = measured_angle - smoothed_angles_[j];
                if (diff > M_PI) diff -= 2*M_PI;
                if (diff < -M_PI) diff += 2*M_PI;
                smoothed_angles_[j] += cfg_.blade_angle_smooth * diff;
            } else {
                smoothed_angles_[j] = measured_angle;
                angle_initialized_[j] = true;
            }
            trackers_[j].angle = smoothed_angles_[j];
        } else {
            trackers_[j].lost = true;
            trackers_[j].lost_count++;
            if (trackers_[j].lost_count > cfg_.max_lost_frames) {
                angle_initialized_[j] = false;
                trackers_[j].lost_count = 0;
            }
        }
    }
}

void Tracker::predictMissing(double dt) {
    float angular_vel_rad = cfg_.angular_velocity_deg * M_PI / 180.0;
    for (int j = 0; j < cfg_.num_blades; ++j) {
        if (trackers_[j].lost && angle_initialized_[j]) {
            smoothed_angles_[j] += angular_vel_rad * dt;
            while (smoothed_angles_[j] > M_PI) smoothed_angles_[j] -= 2*M_PI;
            while (smoothed_angles_[j] < -M_PI) smoothed_angles_[j] += 2*M_PI;
            trackers_[j].angle = smoothed_angles_[j];
            trackers_[j].center = cv::Point2f(smoothed_center_.x + smoothed_radius_ * cos(smoothed_angles_[j]),
                                              smoothed_center_.y + smoothed_radius_ * sin(smoothed_angles_[j]));
        }
    }
}

TrackerResult Tracker::generateResult(const cv::Point2f& center, float radius) {
    TrackerResult res;
    res.model_center = center;
    res.model_radius = radius;
    res.valid = (radius > 0 && !trackers_.empty() && angle_initialized_[0]);
    if (!res.valid) return res;
    res.blades.resize(cfg_.num_blades);
    for (int i = 0; i < cfg_.num_blades; ++i) {
        float angle;
        if (angle_initialized_[i]) {
            angle = smoothed_angles_[i];
        } else if (angle_initialized_[0]) {
            float step = 2 * M_PI / cfg_.num_blades;
            angle = smoothed_angles_[0] + i * step;
        } else {
            continue;
        }
        res.blades[i].center = cv::Point2f(center.x + radius * cos(angle),
                                           center.y + radius * sin(angle));
        res.blades[i].id = i;
        res.blades[i].angle_rad = angle;
        float sz = 50.0f;
        res.blades[i].object_points[0] = cv::Point2f(res.blades[i].center.x + 10, res.blades[i].center.y - sz);
        res.blades[i].object_points[1] = cv::Point2f(res.blades[i].center.x - sz + 10, res.blades[i].center.y);
        res.blades[i].object_points[2] = cv::Point2f(res.blades[i].center.x + 10, res.blades[i].center.y + sz);
        res.blades[i].object_points[3] = cv::Point2f(res.blades[i].center.x + sz + 10, res.blades[i].center.y);
    }
    return res;
}

TrackerResult Tracker::update(const DetectorResult& det, double dt) {
    if (dt > 0.1) dt = 0.033;
    if (dt <= 0) dt = 0.033;

    cv::Point2f new_center = det.r_center;
    float new_radius = 0;
    if (det.r_found && !det.blade_centers.empty()) {
        float sum_dist = 0;
        for (const auto& pt : det.blade_centers) {
            sum_dist += cv::norm(pt - new_center);
        }
        new_radius = sum_dist / det.blade_centers.size();
    } else if (det.r_found) {
        new_radius = smoothed_radius_ > 0 ? smoothed_radius_ : det.r_radius;
    }

    if (first_frame_) {
        smoothed_center_ = new_center;
        smoothed_radius_ = new_radius;

        if (!det.blade_centers.empty()) {
            std::vector<std::pair<float, cv::Point2f>> angle_pts;
            for (const auto& pt : det.blade_centers) {
                float dx = pt.x - smoothed_center_.x;
                float dy = pt.y - smoothed_center_.y;
                float angle = atan2(dy, dx);
                angle_pts.push_back({angle, pt});
            }
            // 自定义排序，只按角度比较
            std::sort(angle_pts.begin(), angle_pts.end(),
                      [](const std::pair<float, cv::Point2f>& a,
                         const std::pair<float, cv::Point2f>& b) {
                          return a.first < b.first;
                      });
            int num = std::min((int)angle_pts.size(), cfg_.num_blades);
            for (int i = 0; i < num; ++i) {
                smoothed_angles_[i] = angle_pts[i].first;
                angle_initialized_[i] = true;
                trackers_[i].center = angle_pts[i].second;
                trackers_[i].angle = angle_pts[i].first;
                trackers_[i].lost = false;
                trackers_[i].lost_count = 0;
            }
        }
        has_ref_angle_ = true;
        last_ref_angle_ = (angle_initialized_[0] ? smoothed_angles_[0] : 0);
        predicted_ref_angle_ = last_ref_angle_;

        first_frame_ = false;
        return generateResult(smoothed_center_, smoothed_radius_);
    }

    if (det.r_found) {
        smoothed_center_ = cfg_.center_smooth * new_center + (1 - cfg_.center_smooth) * smoothed_center_;
        smoothed_radius_ = cfg_.radius_smooth * new_radius + (1 - cfg_.radius_smooth) * smoothed_radius_;
    }

    updateTrackers(det.blade_centers, dt);
    predictMissing(dt);

    bool any_active = false;
    for (int j = 0; j < cfg_.num_blades; ++j) {
        if (!trackers_[j].lost && angle_initialized_[j]) {
            any_active = true;
            break;
        }
    }
    if (any_active) {
        has_ref_angle_ = true;
        for (int j = 0; j < cfg_.num_blades; ++j) {
            if (!trackers_[j].lost && angle_initialized_[j]) {
                last_ref_angle_ = smoothed_angles_[j];
                predicted_ref_angle_ = last_ref_angle_;
                break;
            }
        }
    } else if (has_ref_angle_) {
        float ang_vel = cfg_.angular_velocity_deg * M_PI / 180.0;
        predicted_ref_angle_ += ang_vel * dt;
        if (predicted_ref_angle_ > M_PI) predicted_ref_angle_ -= 2*M_PI;
        if (predicted_ref_angle_ < -M_PI) predicted_ref_angle_ += 2*M_PI;
        last_ref_angle_ = predicted_ref_angle_;
    } else {
        has_ref_angle_ = false;
    }

    last_timestamp_ += dt;
    return generateResult(smoothed_center_, smoothed_radius_);
}

} // namespace energy
