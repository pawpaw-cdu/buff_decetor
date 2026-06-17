#ifndef PNP_SOLVER_HPP
#define PNP_SOLVER_HPP

namespace energy{

#define ENERGY_LENGTH 382   //单位：mm
#define ENERGY_WIDTH 320

class solver
{
private:
    cv::Point3f energy_points[4] = {
        {0, -ENERGY_WIDTH / 2.0, ENERGY_LENGTH / 2.0},
        {0, ENERGY_WIDTH / 2.0, ENERGY_LENGTH / 2.0},
        {0, ENERGY_WIDTH / 2.0, -ENERGY_LENGTH / 2.0},
        {0, -ENERGY_WIDTH / 2.0, -ENERGY_LENGTH / 2.0}
    };
    
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    double result;
public:
    solver(const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
    : camera_matrix_(camera_matrix.clone()),dist_coeffs_(dist_coeffs.clone()){}

    ~solver();

    void getSolveResult(){return result};

    bool solvePnP(const EnergyBlade& blade, cv::Mat& tvec, cv::Mat& rvec);
};

solver::solver(/* args */)
{

}

solver::~solver()
{
}



}


#endif

