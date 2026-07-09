#include "oxebots_prediction/kalman_filter_node.hpp"

#include "Eigen/src/Core/Matrix.h"
#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"

/*
 * Vamos utilizar um filtro de Kalman Extendido para realizar a predicao do
 * movimento dos objetos. Para isso, vamos utilizar um sistema com uma
 * aceleracao igual a uma funcao do primeiro grau:
 *
 * a_k = \alpha * t + \beta
 *
 * Assim, podemos integrar a funcao para termos uma funcao da velocidade:
 *
 * v_k = (1/2) * \alpha * t^2 + \beta * t + \gamma
 *
 * E, da mesma maneira, integrar novamente para termos a posicao:
 *
 * p_k = (1/6) * \alpha * t^3 + (1/2) * \beta * t^2 + \gamma * t + \psi
 *
 * Asim, vamos ter um estado definido como:
 *
 * x = [p_kx, p_ky, v_kx, v_ky, a_kx, a_ky, \alpha_kx, \alpha_ky]
 */

KalmanFilterNode::KalmanFilterNode() : rclcpp::Node("oxebots_kalman_filter")
{
    declare_parameter("ball_publisher", "ball_predicted");
    declare_parameter("ball_subscriber", "ball_data");
    declare_parameter("robot_publisher", "robot_predicted");
    declare_parameter("robot_subscriber", "robot_data");
    declare_parameter("maximum_publisher_size", 10);
    declare_parameter("team_size", 3);
    declare_parameter("covariance_constant_multiplier", 1000.0);
    declare_parameter("sensor_measurement_noise", 1.0f);
    declare_parameter("process_noise", 0.8f);

    _ball_publisher = create_publisher<oxebots_interfaces::msg::BallPrediction>(
      get_parameter("ball_publisher").as_string(),
      get_parameter("maximum_publisher_size").as_int());

    _robots_publisher =
      create_publisher<oxebots_interfaces::msg::RobotPrediction>(
        get_parameter("robot_publisher").as_string(),
        get_parameter("maximum_publisher_size").as_int());

    _robots_subscriber =
      create_subscription<oxebots_interfaces::msg::RobotPosition>(
        get_parameter("robot_subscriber").as_string(),
        get_parameter("maximum_publisher_size").as_int(),
        std::bind(&KalmanFilterNode::robots_data_callback, this,
                  std::placeholders::_1));

    _ball_subscriber =
      create_subscription<oxebots_interfaces::msg::BallPosition>(
        get_parameter("ball_subscriber").as_string(),
        get_parameter("maximum_publisher_size").as_int(),
        std::bind(&KalmanFilterNode::ball_data_callback, this,
                  std::placeholders::_1));

    for (int i = 0; i < get_parameter("team_size").as_int(); i++)
    {
        RobotControlState ally_control;
        ally_control.state = Eigen::VectorXd::Zero(8);
        ally_control.id = i;
        ally_control.initialized = false;
        _allies_state.push_back(ally_control);

        RobotControlState enemy_control;
        enemy_control.state = Eigen::VectorXd::Zero(8);
        enemy_control.id = i;
        enemy_control.initialized = false;
        _enemies_state.push_back(enemy_control);

        RobotCovarianceState ally_cov;
        ally_cov.id = i;
        ally_cov.covariance = Eigen::MatrixXd::Identity(8, 8) *
          get_parameter("covariance_constant_multiplier").as_double();
        _allies_covariance.push_back(ally_cov);

        RobotCovarianceState enemy_cov;
        enemy_cov.id = i;
        enemy_cov.covariance = Eigen::MatrixXd::Identity(8, 8) *
          get_parameter("covariance_constant_multiplier").as_double();

        _enemies_covariance.push_back(enemy_cov);
    }

    _ball_state = Eigen::VectorXd(8);
    _ball_covariance = Eigen::MatrixXd::Zero(8, 8);

    _is_ball_initialized = false;
    _is_robots_initialized = false;

    _measurement_matrix = Eigen::MatrixXd(Eigen::MatrixXd::Zero(2, 8));
    _measurement_matrix(0, 0) = 1;
    _measurement_matrix(1, 1) = 1;

    double sensor_measurment_squared =
      get_parameter("sensor_measurement_noise").as_double() *
      get_parameter("sensor_measurement_noise").as_double();

    _measurement_noise << sensor_measurment_squared, 0,
                          0, sensor_measurment_squared;

    _process_noise = Eigen::MatrixXd::Identity(8, 8) *
                     get_parameter("process_noise").as_double();
}

KalmanFilterNode::~KalmanFilterNode()
{
}

void KalmanFilterNode::initialize_ball_filter(
  oxebots_interfaces::msg::BallPosition msg)
{
    // Set the initial values
    _ball_state.setZero();
    _ball_state(0) = msg.x;
    _ball_state(1) = msg.y;
    _ball_covariance = Eigen::MatrixXd::Identity(8, 8);
    _last_ball_time = now();
    RCLCPP_INFO(get_logger(), "Initializing the Ball filter...");
}



void KalmanFilterNode::update_ball_filter(
  oxebots_interfaces::msg::BallPosition msg)
{
    RCLCPP_DEBUG(get_logger(), "Updating the ball filter");
    Eigen::VectorXd sensor_data = Eigen::VectorXd(2);
    sensor_data << msg.x, msg.y;

    Eigen::MatrixXd innovation_covariance_matrix =
      _measurement_matrix * _ball_covariance *
        _measurement_matrix.transpose() +
      _measurement_noise;

    Eigen::MatrixXd kalman_gain = _ball_covariance *
                                  _measurement_matrix.transpose() *
                                  innovation_covariance_matrix.inverse();

    Eigen::VectorXd y = sensor_data - _measurement_matrix * _ball_state;
    _ball_state += kalman_gain * y;
    _ball_covariance =
      (Eigen::MatrixXd::Identity(8, 8) - kalman_gain * _measurement_matrix) *
      _ball_covariance;
}

void KalmanFilterNode::update_robots_filter(
  oxebots_interfaces::msg::RobotPosition msg)
{
    auto update_list = [&](const std::vector<oxebots_interfaces::msg::RobotGameData>& robots, 
                           std::vector<RobotControlState>& states, 
                           std::vector<RobotCovarianceState>& covariances) {
        for (const auto& robot : robots) {
            if (robot.id >= states.size()) continue;

            auto& state_obj = states[robot.id];
            auto& cov_obj = covariances[robot.id];

            if (!state_obj.initialized) {
                state_obj.state.setZero();
                state_obj.state(0) = robot.x;
                state_obj.state(1) = robot.y;
                state_obj.state(6) = robot.orientation;
                state_obj.initialized = true;
                continue;
            }

            // Measurement: x, y, orientation
            Eigen::VectorXd z(3);
            z << robot.x, robot.y, robot.orientation;

            // Handle orientation wrap-around
            double diff_theta = z(2) - state_obj.state(6);
            while (diff_theta > M_PI) diff_theta -= 2 * M_PI;
            while (diff_theta < -M_PI) diff_theta += 2 * M_PI;
            z(2) = state_obj.state(6) + diff_theta;

            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 8);
            H(0, 0) = 1; H(1, 1) = 1; H(2, 6) = 1;

            Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * get_parameter("sensor_measurement_noise").as_double();
            
            Eigen::MatrixXd S = H * cov_obj.covariance * H.transpose() + R;
            Eigen::MatrixXd K = cov_obj.covariance * H.transpose() * S.inverse();

            state_obj.state += K * (z - H * state_obj.state);
            cov_obj.covariance = (Eigen::MatrixXd::Identity(8, 8) - K * H) * cov_obj.covariance;
            
            // Normalize orientation after update
            while (state_obj.state(6) > M_PI) state_obj.state(6) -= 2 * M_PI;
            while (state_obj.state(6) < -M_PI) state_obj.state(6) += 2 * M_PI;
        }
    };

    update_list(msg.allies, _allies_state, _allies_covariance);
    update_list(msg.enemies, _enemies_state, _enemies_covariance);
}

void KalmanFilterNode::prediction_ball_filter()
{
    rclcpp::Time current_time = now();
    double dt = (current_time - _last_ball_time).seconds();
    _last_ball_time = current_time;

    if (dt <= 0) return;

    RCLCPP_DEBUG(get_logger(), "Ball prediction with dt: %f", dt);

    // Update state using the motion model (p_k = p_k-1 + v*dt + 0.5*a*dt^2 + ...)
    Eigen::VectorXd next_state(8);
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;

    next_state(0) = _ball_state(0) + _ball_state(2) * dt + 0.5 * _ball_state(4) * dt2 + (1.0 / 6.0) * _ball_state(6) * dt3;
    next_state(1) = _ball_state(1) + _ball_state(3) * dt + 0.5 * _ball_state(5) * dt2 + (1.0 / 6.0) * _ball_state(7) * dt3;
    next_state(2) = _ball_state(2) + _ball_state(4) * dt + 0.5 * _ball_state(6) * dt2;
    next_state(3) = _ball_state(3) + _ball_state(5) * dt + 0.5 * _ball_state(7) * dt2;
    next_state(4) = _ball_state(4) + _ball_state(6) * dt;
    next_state(5) = _ball_state(5) + _ball_state(7) * dt;
    next_state(6) = _ball_state(6);
    next_state(7) = _ball_state(7);

    _ball_state = next_state;

    // Jacobian matrix for the motion model
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(8, 8);
    jacobian(0, 2) = dt; jacobian(0, 4) = 0.5 * dt2; jacobian(0, 6) = (1.0 / 6.0) * dt3;
    jacobian(1, 3) = dt; jacobian(1, 5) = 0.5 * dt2; jacobian(1, 7) = (1.0 / 6.0) * dt3;
    jacobian(2, 4) = dt; jacobian(2, 6) = 0.5 * dt2;
    jacobian(3, 5) = dt; jacobian(3, 7) = 0.5 * dt2;
    jacobian(4, 6) = dt;
    jacobian(5, 7) = dt;

    _ball_covariance = jacobian * _ball_covariance * jacobian.transpose() + _process_noise;

    // Publish prediction
    oxebots_interfaces::msg::BallPrediction prediction_msg;
    prediction_msg.x = _ball_state(0);
    prediction_msg.y = _ball_state(1);
    prediction_msg.vx = _ball_state(2);
    prediction_msg.vy = _ball_state(3);
    prediction_msg.ax = _ball_state(4);
    prediction_msg.ay = _ball_state(5);
    _ball_publisher->publish(prediction_msg);
}

void KalmanFilterNode::prediction_robots_filter()
{
    rclcpp::Time current_time = now();
    double dt = (current_time - _last_robots_time).seconds();
    _last_robots_time = current_time;

    if (dt <= 0) return;

    auto predict_list = [&](std::vector<RobotControlState>& states, 
                            std::vector<RobotCovarianceState>& covariances) -> std::vector<oxebots_interfaces::msg::RobotPredictionData> {
        std::vector<oxebots_interfaces::msg::RobotPredictionData> predicted_robots;
        
        double dt2 = dt * dt;
        
        // Jacobian for motion model
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(8, 8);
        F(0, 2) = dt; F(0, 4) = 0.5 * dt2;
        F(1, 3) = dt; F(1, 5) = 0.5 * dt2;
        F(2, 4) = dt;
        F(3, 5) = dt;
        F(6, 7) = dt;

        for (size_t i = 0; i < states.size(); ++i) {
            if (!states[i].initialized) continue;

            auto& state = states[i].state;
            auto& cov = covariances[i].covariance;

            // State prediction
            state = F * state;

            // Normalize orientation
            while (state(6) > M_PI) state(6) -= 2 * M_PI;
            while (state(6) < -M_PI) state(6) += 2 * M_PI;

            // Covariance prediction
            cov = F * cov * F.transpose() + _process_noise;

            // Prepare message
            oxebots_interfaces::msg::RobotPredictionData data;
            data.id = states[i].id;
            data.x = state(0);
            data.y = state(1);
            data.vx = state(2);
            data.vy = state(3);
            data.ax = state(4);
            data.ay = state(5);
            data.orientation = state(6);
            data.vorientation = state(7);
            predicted_robots.push_back(data);
        }
        return predicted_robots;
    };

    oxebots_interfaces::msg::RobotPrediction predicted_msg;
    predicted_msg.allies = predict_list(_allies_state, _allies_covariance);
    predicted_msg.enemies = predict_list(_enemies_state, _enemies_covariance);
    _robots_publisher->publish(predicted_msg);
}

void KalmanFilterNode::ball_data_callback(
  oxebots_interfaces::msg::BallPosition msg)
{
    if (!_is_ball_initialized)
    {
        _is_ball_initialized = true;
        initialize_ball_filter(msg);
        return;
    }

    prediction_ball_filter();
    update_ball_filter(msg);
}

void KalmanFilterNode::robots_data_callback(
  oxebots_interfaces::msg::RobotPosition msg)
{
    if (!_is_robots_initialized)
    {
        _is_robots_initialized = true;
        _last_robots_time = now();
        RCLCPP_INFO(get_logger(), "Initializing the Robots filter...");
    }

    update_robots_filter(msg);
    prediction_robots_filter();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
