#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace oxebots_prediction
{

struct RobotFilterConfig
{
    double pos_measurement_noise = 3.0;      // Desvio padrão do ruído de posição (mm)
    double angle_measurement_noise = 0.02;   // Desvio padrão do ruído de orientação (rad ~ 1.15 deg)
    double pos_process_noise = 4000.0;       // Incerteza de aceleração linear (mm/s^2 ~ 4 m/s^2)
    double angle_process_noise = 30.0;       // Incerteza de aceleração angular (rad/s^2)
};

class RobotKalmanFilter
{
public:
    explicit RobotKalmanFilter(const RobotFilterConfig & config = RobotFilterConfig());

    void init(double x, double y, double orientation);
    void predict(double dt);
    void update(double meas_x, double meas_y, double meas_orientation);

    double x() const { return state_(0); }
    double y() const { return state_(1); }
    double vx() const { return state_(2); }
    double vy() const { return state_(3); }
    double orientation() const { return state_(4); }
    double angular_velocity() const { return state_(5); }

    const Eigen::Matrix<double, 6, 1> & state() const { return state_; }
    const Eigen::Matrix<double, 6, 6> & covariance() const { return covariance_; }

    bool is_initialized() const { return initialized_; }

    static double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

private:
    RobotFilterConfig config_;
    Eigen::Matrix<double, 6, 1> state_;       // [x, y, vx, vy, theta, omega]
    Eigen::Matrix<double, 6, 6> covariance_;  // P
    bool initialized_{false};
};

} // namespace oxebots_prediction
