#include "oxebots_prediction/kalman_filter_node.hpp"

#include <functional>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Macros.h"
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
    declare_parameter("ball_publisher", "ball_publisher");
    declare_parameter("ball_subscriber", "ball_subscriber");
    declare_parameter("robot_publisher", "robot_publisher");
    declare_parameter("robot_subscriber", "robot_subscriber");
    declare_parameter("maximum_publisher_size", 10);
    declare_parameter("team_size", 4);
    declare_parameter("covariance_constant_multiplier", 1000);

    _ball_publisher = create_publisher<oxebots_interfaces::msg::BallPredicted>(
      get_parameter("ball_publisher").as_string(),
      get_parameter("maximum_publisher_size").as_int());

    _robots_publisher =
      create_publisher<oxebots_interfaces::msg::RobotPredicted>(
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

    _allies_state = new std::vector<RobotControlState>();
    _enemies_state = new std::vector<RobotControlState>();
    _allies_covariance = new std::vector<RobotCovarianceState>();
    _enemies_covariance = new std::vector<RobotCovarianceState>();

    for (int i = 0; i < get_parameter("team_size").as_int(); i++)
    {
        RobotControlState ally_control;
        ally_control.state = new Eigen::VectorXd(8);
        ally_control.id = i;
        _allies_state->push_back(ally_control);

        RobotControlState enemy_control;
        enemy_control.state = new Eigen::VectorXd(8);
        enemy_control.id = i;
        _enemies_state->push_back(enemy_control);

        RobotCovarianceState ally_cov;
        ally_cov.id = i;
        ally_cov.covariance = new Eigen::MatrixXd(
          Eigen::MatrixXd::Identity(8, 8) *
          get_parameter("covariance_constant_multiplier").as_double());
        _allies_covariance->push_back(ally_cov);

        RobotCovarianceState enemy_cov;
        enemy_cov.id = i;
        enemy_cov.covariance = new Eigen::MatrixXd(
          Eigen::MatrixXd::Identity(8, 8) *
          get_parameter("covariance_constant_multiplier").as_double());

        _enemies_covariance->push_back(enemy_cov);
    }

    _ball_state = new Eigen::VectorXd(8);
    _ball_covariance = new Eigen::MatrixXd(Eigen::MatrixXd::Zero(8, 8));
}

KalmanFilterNode::~KalmanFilterNode() {}

void KalmanFilterNode::initialize_ball_filter() {}

void KalmanFilterNode::initialize_robots_filter() {}

void KalmanFilterNode::update_ball_filter() {}

void KalmanFilterNode::update_robots_filter() {}

void KalmanFilterNode::prediction_ball_filter() {}

void KalmanFilterNode::prediction_robots_filter() {}

void KalmanFilterNode::ball_data_callback(
  oxebots_interfaces::msg::BallPosition)
{
}

void KalmanFilterNode::robots_data_callback(
  oxebots_interfaces::msg::RobotPosition)
{
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
