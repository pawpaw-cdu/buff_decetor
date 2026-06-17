#include <vector>
#include <opencv2/opencv.hpp>
#include "../include/pnp_solver.hpp"

namespace energy{

bool solver::solvePnP(const EnergyBlade& blade, cv::Mat& tvec, cv::Mat& rvec){
        std::vector<cv::Point2f> img_energy_points;

        img_energy_points.emplace_back(blade.object_points[0]);
        img_energy_points.emplace_back(blade.object_points[1]);
        img_energy_points.emplace_back(blade.object_points[2]);
        img_energy_points.emplace_back(blade.object_points[3]);

        return cv::solvePnP(energy_points, img_energy_points, 
                camera_matrix_, dist_coeffs_, rvec, tvec, false,
                cv::SOLVEPNP_IPPE);
    }
}