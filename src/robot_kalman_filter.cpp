#include "oxebots_prediction/robot_kalman_filter.hpp"
#include <algorithm>

namespace oxebots_prediction
{

RobotKalmanFilter::RobotKalmanFilter(const RobotFilterConfig & config)
: config_(config),
  state_(Eigen::Matrix<double, 6, 1>::Zero()),
  covariance_(Eigen::Matrix<double, 6, 6>::Identity() * 1000.0),
  initialized_(false)
{
}

void RobotKalmanFilter::init(double x, double y, double orientation)
{
    state_ << x, y, 0.0, 0.0, normalize_angle(orientation), 0.0;

    covariance_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_(0, 0) = config_.pos_measurement_noise * config_.pos_measurement_noise;
    covariance_(1, 1) = config_.pos_measurement_noise * config_.pos_measurement_noise;
    covariance_(2, 2) = 1000000.0; // Incerteza inicial de velocidade translacional (1000 mm/s)
    covariance_(3, 3) = 1000000.0;
    covariance_(4, 4) = config_.angle_measurement_noise * config_.angle_measurement_noise;
    covariance_(5, 5) = 100.0;     // Incerteza inicial de velocidade angular (10 rad/s)
    initialized_ = true;
}

void RobotKalmanFilter::predict(double dt)
{
    if (!initialized_ || dt <= 0.0) {
        return;
    }

    dt = std::min(dt, 0.2);

    // Modelo cinemático de transição (omni desacoplado em plano global)
    // x_k = x_{k-1} + v_x * dt
    // y_k = y_{k-1} + v_y * dt
    // theta_k = theta_{k-1} + omega * dt
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;
    F(4, 5) = dt;

    state_ = F * state_;
    state_(4) = normalize_angle(state_(4));

    // Ruído de processo Q por blocos translacionais e rotacionais
    double q_pos_var = config_.pos_process_noise * config_.pos_process_noise;
    double q_ang_var = config_.angle_process_noise * config_.angle_process_noise;

    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();

    // Bloco X
    Q(0, 0) = q_pos_var * (dt4 / 4.0);
    Q(0, 2) = q_pos_var * (dt3 / 2.0);
    Q(2, 0) = q_pos_var * (dt3 / 2.0);
    Q(2, 2) = q_pos_var * dt2;

    // Bloco Y
    Q(1, 1) = q_pos_var * (dt4 / 4.0);
    Q(1, 3) = q_pos_var * (dt3 / 2.0);
    Q(3, 1) = q_pos_var * (dt3 / 2.0);
    Q(3, 3) = q_pos_var * dt2;

    // Bloco Theta
    Q(4, 4) = q_ang_var * (dt4 / 4.0);
    Q(4, 5) = q_ang_var * (dt3 / 2.0);
    Q(5, 4) = q_ang_var * (dt3 / 2.0);
    Q(5, 5) = q_ang_var * dt2;

    covariance_ = F * covariance_ * F.transpose() + Q;
}

void RobotKalmanFilter::update(double meas_x, double meas_y, double meas_orientation)
{
    if (!initialized_) {
        init(meas_x, meas_y, meas_orientation);
        return;
    }

    // Inovação com normalização de ângulo
    Eigen::Vector3d y;
    y(0) = meas_x - state_(0);
    y(1) = meas_y - state_(1);
    y(2) = normalize_angle(meas_orientation - state_(4));

    // Matriz de medição H (3 x 6) observando [x, y, theta]
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 4) = 1.0;

    // Matriz de ruído de medição R com unidades devidamente desacopladas
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R(0, 0) = config_.pos_measurement_noise * config_.pos_measurement_noise;
    R(1, 1) = config_.pos_measurement_noise * config_.pos_measurement_noise;
    R(2, 2) = config_.angle_measurement_noise * config_.angle_measurement_noise;

    Eigen::Matrix3d S = H * covariance_ * H.transpose() + R;
    Eigen::Matrix<double, 6, 3> K = covariance_ * H.transpose() * S.inverse();

    state_ += K * y;
    state_(4) = normalize_angle(state_(4));

    Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> I_KH = I - K * H;

    // Forma simétrica e numericamente estável de Joseph
    covariance_ = I_KH * covariance_ * I_KH.transpose() + K * R * K.transpose();
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

} // namespace oxebots_prediction
