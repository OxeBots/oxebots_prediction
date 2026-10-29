#include <gtest/gtest.h>
#include <cmath>
#include "oxebots_prediction/ball_kalman_filter.hpp"
#include "oxebots_prediction/robot_kalman_filter.hpp"

using namespace oxebots_prediction;

// ============================================================================
// Testes do Filtro de Kalman da Bola (BallKalmanFilter)
// ============================================================================

TEST(BallKalmanFilterTest, Initialization)
{
    BallKalmanFilter filter;
    EXPECT_FALSE(filter.is_initialized());

    filter.init(100.0, -200.0);
    EXPECT_TRUE(filter.is_initialized());
    EXPECT_DOUBLE_EQ(filter.x(), 100.0);
    EXPECT_DOUBLE_EQ(filter.y(), -200.0);
    EXPECT_DOUBLE_EQ(filter.vx(), 0.0);
    EXPECT_DOUBLE_EQ(filter.vy(), 0.0);
}

TEST(BallKalmanFilterTest, StaticBallStaysAtRest)
{
    BallFilterConfig config;
    config.friction_accel = 800.0;
    BallKalmanFilter filter(config);

    filter.init(0.0, 0.0);

    // Simula 20 frames a 60 Hz (dt ~ 0.0167s) com bola imóvel
    const double dt = 1.0 / 60.0;
    for (int i = 0; i < 20; ++i) {
        filter.predict(dt);
        filter.update(0.0, 0.0, dt);
    }

    EXPECT_NEAR(filter.x(), 0.0, 0.1);
    EXPECT_NEAR(filter.y(), 0.0, 0.1);
    EXPECT_NEAR(filter.vx(), 0.0, 1.0);
    EXPECT_NEAR(filter.vy(), 0.0, 1.0);
}

TEST(BallKalmanFilterTest, FrictionDeceleratesRollingBall)
{
    BallFilterConfig config;
    config.friction_accel = 800.0; // 800 mm/s^2
    BallKalmanFilter filter(config);

    filter.init(0.0, 0.0);

    // Simula bola rolando inicialmente a 1000 mm/s no eixo X
    double sim_x = 0.0;
    double sim_vx = 1000.0;
    const double dt = 0.016;

    // 10 passos para o filtro convergir à velocidade
    for (int i = 0; i < 10; ++i) {
        sim_x += sim_vx * dt;
        filter.predict(dt);
        filter.update(sim_x, 0.0, dt);
    }

    double speed_before = std::hypot(filter.vx(), filter.vy());
    EXPECT_GT(speed_before, 500.0);

    // Deixa apenas com predict para avaliar o modelo físico puro de atrito com carpete
    for (int i = 0; i < 30; ++i) {
        filter.predict(dt);
    }

    double speed_after = std::hypot(filter.vx(), filter.vy());
    // A velocidade deve ter diminuído expressivamente devido à desaceleração por atrito
    EXPECT_LT(speed_after, speed_before);
    EXPECT_LE(filter.ax(), 0.0); // Desaceleração contrária ao movimento
}

TEST(BallKalmanFilterTest, KickDetectionAdaptsVelocityQuickly)
{
    BallFilterConfig config;
    config.kick_threshold_mm = 80.0;
    BallKalmanFilter filter(config);

    filter.init(0.0, 0.0);
    const double dt = 0.016;

    // Bola em repouso
    for (int i = 0; i < 10; ++i) {
        filter.predict(dt);
        filter.update(0.0, 0.0, dt);
    }

    // Chute repentino: salto de 150 mm no frame seguinte (> kick_threshold_mm)
    filter.predict(dt);
    filter.update(150.0, 0.0, dt);

    // Deve detectar o chute e estimar velocidade positiva substancial no eixo X imediatamente
    EXPECT_GT(filter.vx(), 2000.0);
    EXPECT_NEAR(filter.x(), 150.0, 5.0);
}

TEST(BallKalmanFilterTest, TeleportResetsFilterPositionWithoutVelocitySpike)
{
    BallKalmanFilter filter;
    filter.init(0.0, 0.0);
    const double dt = 0.016;

    // Bola em repouso
    filter.predict(dt);
    filter.update(0.0, 0.0, dt);

    // Teletransporte de 1000 mm (> 400 mm kTeleportThresholdMm) comum em resets de simulação
    filter.predict(dt);
    filter.update(1000.0, 500.0, dt);

    // O filtro deve reinicializar na nova posição sem gerar velocidades de 50.000 mm/s
    EXPECT_DOUBLE_EQ(filter.x(), 1000.0);
    EXPECT_DOUBLE_EQ(filter.y(), 500.0);
    EXPECT_DOUBLE_EQ(filter.vx(), 0.0);
    EXPECT_DOUBLE_EQ(filter.vy(), 0.0);
}

// ============================================================================
// Testes do Filtro de Kalman do Robô (RobotKalmanFilter)
// ============================================================================

TEST(RobotKalmanFilterTest, Initialization)
{
    RobotKalmanFilter filter;
    EXPECT_FALSE(filter.is_initialized());

    filter.init(500.0, -300.0, M_PI_2);
    EXPECT_TRUE(filter.is_initialized());
    EXPECT_DOUBLE_EQ(filter.x(), 500.0);
    EXPECT_DOUBLE_EQ(filter.y(), -300.0);
    EXPECT_NEAR(filter.orientation(), M_PI_2, 1e-6);
    EXPECT_DOUBLE_EQ(filter.vx(), 0.0);
    EXPECT_DOUBLE_EQ(filter.vy(), 0.0);
    EXPECT_DOUBLE_EQ(filter.angular_velocity(), 0.0);
}

TEST(RobotKalmanFilterTest, NormalizeAngle)
{
    EXPECT_NEAR(RobotKalmanFilter::normalize_angle(0.0), 0.0, 1e-6);
    EXPECT_NEAR(RobotKalmanFilter::normalize_angle(M_PI), M_PI, 1e-6);
    EXPECT_NEAR(RobotKalmanFilter::normalize_angle(3.0 * M_PI), M_PI, 1e-6);
    EXPECT_NEAR(RobotKalmanFilter::normalize_angle(-3.0 * M_PI), -M_PI, 1e-6);
    EXPECT_NEAR(RobotKalmanFilter::normalize_angle(2.5 * M_PI), M_PI_2, 1e-6);
}

TEST(RobotKalmanFilterTest, ConstantLinearVelocityTracking)
{
    RobotFilterConfig config;
    RobotKalmanFilter filter(config);

    filter.init(0.0, 0.0, 0.0);
    const double dt = 0.0166;
    const double target_vx = 500.0; // 500 mm/s (0.5 m/s)

    double sim_x = 0.0;
    for (int i = 0; i < 30; ++i) {
        sim_x += target_vx * dt;
        filter.predict(dt);
        filter.update(sim_x, 0.0, 0.0);
    }

    EXPECT_NEAR(filter.x(), sim_x, 5.0);
    EXPECT_NEAR(filter.vx(), target_vx, 30.0);
    EXPECT_NEAR(filter.vy(), 0.0, 10.0);
}

TEST(RobotKalmanFilterTest, AngularContinuityAcrossBranchCut)
{
    RobotFilterConfig config;
    RobotKalmanFilter filter(config);

    // Inicia próximo de +pi (170 graus)
    const double initial_angle = 170.0 * M_PI / 180.0;
    filter.init(0.0, 0.0, initial_angle);

    const double dt = 0.0166;
    const double target_omega = 2.0; // 2 rad/s positivo (gira anti-horário e cruza PI para -PI)

    double current_angle = initial_angle;
    for (int i = 0; i < 15; ++i) {
        current_angle = RobotKalmanFilter::normalize_angle(current_angle + target_omega * dt);
        filter.predict(dt);
        filter.update(0.0, 0.0, current_angle);
    }

    // A velocidade angular estimada deve ser positiva (~2.0 rad/s) e não ter saltos negativos gigantes
    EXPECT_GT(filter.angular_velocity(), 0.5);
    EXPECT_NEAR(filter.angular_velocity(), target_omega, 0.5);
}
