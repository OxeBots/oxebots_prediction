#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace oxebots_prediction
{

struct BallFilterConfig
{
    double pos_measurement_noise = 3.0;     // Desvio padrão do ruído do sensor de posição (mm)
    double process_noise = 2000.0;          // Incerteza de aceleração de processo (mm/s^2)
    double friction_accel = 800.0;          // Desaceleração por atrito de rolamento no carpete (\mu * g) em mm/s^2
    double kick_threshold_mm = 80.0;        // Inovação mínima para disparo de detecção de chute/impacto
};

class BallKalmanFilter
{
public:
    explicit BallKalmanFilter(const BallFilterConfig & config = BallFilterConfig());

    void init(double x, double y);
    void predict(double dt);
    void update(double meas_x, double meas_y, double dt);

    double x() const { return state_(0); }
    double y() const { return state_(1); }
    double vx() const { return state_(2); }
    double vy() const { return state_(3); }
    double ax() const { return ax_; }
    double ay() const { return ay_; }

    const Eigen::Vector4d & state() const { return state_; }
    const Eigen::Matrix4d & covariance() const { return covariance_; }

    bool is_initialized() const { return initialized_; }

private:
    BallFilterConfig config_;
    Eigen::Vector4d state_;       // [x, y, vx, vy]
    Eigen::Matrix4d covariance_;  // P
    double ax_{0.0};
    double ay_{0.0};
    bool initialized_{false};
};

} // namespace oxebots_prediction
