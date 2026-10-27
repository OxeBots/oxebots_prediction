#include "oxebots_prediction/ball_kalman_filter.hpp"
#include <algorithm>

namespace oxebots_prediction
{

BallKalmanFilter::BallKalmanFilter(const BallFilterConfig & config)
: config_(config),
  state_(Eigen::Vector4d::Zero()),
  covariance_(Eigen::Matrix4d::Identity() * 1000.0),
  ax_(0.0),
  ay_(0.0),
  initialized_(false)
{
}

void BallKalmanFilter::init(double x, double y)
{
    state_ << x, y, 0.0, 0.0;
    covariance_ = Eigen::Matrix4d::Zero();
    covariance_(0, 0) = config_.pos_measurement_noise * config_.pos_measurement_noise;
    covariance_(1, 1) = config_.pos_measurement_noise * config_.pos_measurement_noise;
    covariance_(2, 2) = 10000.0; // Incerteza inicial de velocidade (100 mm/s)
    covariance_(3, 3) = 10000.0;
    ax_ = 0.0;
    ay_ = 0.0;
    initialized_ = true;
}

void BallKalmanFilter::predict(double dt)
{
    if (!initialized_ || dt <= 0.0) {
        return;
    }

    // Limita dt máximo para evitar saltos numéricos anômalos
    dt = std::min(dt, 0.2);

    double x = state_(0);
    double y = state_(1);
    double vx = state_(2);
    double vy = state_(3);
    double speed = std::hypot(vx, vy);

    // Modelo de atrito de rolamento da bola com o carpete (SSL)
    // a = -\mu * g * (v / ||v||)
    constexpr double kMinRollingSpeed = 30.0; // mm/s
    if (speed > kMinRollingSpeed) {
        ax_ = -config_.friction_accel * (vx / speed);
        ay_ = -config_.friction_accel * (vy / speed);

        double next_vx = vx + ax_ * dt;
        double next_vy = vy + ay_ * dt;

        // Se o atrito parou a bola durante o intervalo dt, não reverte o sentido
        if (next_vx * vx <= 0.0) next_vx = 0.0;
        if (next_vy * vy <= 0.0) next_vy = 0.0;

        state_(0) = x + vx * dt + 0.5 * ax_ * dt * dt;
        state_(1) = y + vy * dt + 0.5 * ay_ * dt * dt;
        state_(2) = next_vx;
        state_(3) = next_vy;
    } else {
        // Bola estática
        ax_ = 0.0;
        ay_ = 0.0;
        state_(2) = 0.0;
        state_(3) = 0.0;
    }

    // Matriz de transição linear F
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    // Ruído de processo Q (Piecewise Continuous White Noise Acceleration)
    double q_var = config_.process_noise * config_.process_noise;
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q(0, 0) = q_var * (dt4 / 4.0);
    Q(0, 2) = q_var * (dt3 / 2.0);
    Q(1, 1) = q_var * (dt4 / 4.0);
    Q(1, 3) = q_var * (dt3 / 2.0);
    Q(2, 0) = q_var * (dt3 / 2.0);
    Q(2, 2) = q_var * dt2;
    Q(3, 1) = q_var * (dt3 / 2.0);
    Q(3, 3) = q_var * dt2;

    covariance_ = F * covariance_ * F.transpose() + Q;
}

void BallKalmanFilter::update(double meas_x, double meas_y, double dt)
{
    if (!initialized_) {
        init(meas_x, meas_y);
        return;
    }

    Eigen::Vector2d z(meas_x, meas_y);
    Eigen::Vector2d y = z - state_.head<2>();

    // Detecção de teleporte no simulador (grSim) vs chute / impacto real:
    // Se a bola saltou mais de 400mm em um único frame (~16ms -> >25 m/s), ela foi reposicionada
    // manualmente no grSim. Reinicializa o filtro na nova posição com velocidade zero para não
    // gerar velocidades absurdas que ficam oscilando de um lado para o outro por ~1-2 segundos.
    double innov_dist = y.norm();
    constexpr double kTeleportThresholdMm = 400.0;
    if (innov_dist > kTeleportThresholdMm) {
        init(meas_x, meas_y);
        return;
    }

    if (innov_dist > config_.kick_threshold_mm) {
        constexpr double kKickCovarianceInflation = 9000000.0; // (3000 mm/s)^2
        covariance_(2, 2) += kKickCovarianceInflation;
        covariance_(3, 3) += kKickCovarianceInflation;

        if (dt > 1e-4) {
            // Ajuste instantâneo da posição e velocidade sem aplicar o salto duas vezes via K*y
            constexpr double kMaxBallSpeedMmS = 8000.0;
            double vx_est = std::clamp(y(0) / dt, -kMaxBallSpeedMmS, kMaxBallSpeedMmS);
            double vy_est = std::clamp(y(1) / dt, -kMaxBallSpeedMmS, kMaxBallSpeedMmS);
            state_(0) = meas_x;
            state_(1) = meas_y;
            state_(2) = 0.5 * state_(2) + 0.5 * vx_est;
            state_(3) = 0.5 * state_(3) + 0.5 * vy_est;
            y = z - state_.head<2>();
        }
    }

    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    double r_var = config_.pos_measurement_noise * config_.pos_measurement_noise;
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * r_var;

    Eigen::Matrix2d S = H * covariance_ * H.transpose() + R;
    Eigen::Matrix<double, 4, 2> K = covariance_ * H.transpose() * S.inverse();

    state_ += K * y;

    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d I_KH = I - K * H;

    // Forma simétrica e estável de Joseph: P = (I - KH) * P * (I - KH)^T + K * R * K^T
    covariance_ = I_KH * covariance_ * I_KH.transpose() + K * R * K.transpose();
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

} // namespace oxebots_prediction
