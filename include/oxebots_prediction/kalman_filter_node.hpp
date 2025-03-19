#pragma once
#include <sys/types.h>

#include <boost/optional.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <cstdint>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Macros.h"
#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/ball_predicted.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"
#include "oxebots_interfaces/msg/robot_predicted.hpp"

typedef struct RobotControlState
{
    Eigen::VectorXd * state;
    uint32_t id;
} RobotControlState;

typedef struct RobotCovarianceState
{
    Eigen::MatrixXd * covariance;
    uint32_t id;
} RobotCovarianceState;

class KalmanFilterNode : public rclcpp::Node
{
   public:
    KalmanFilterNode();
    ~KalmanFilterNode();

   protected:
    void ball_data_callback(oxebots_interfaces::msg::BallPosition msg);
    void robots_data_callback(oxebots_interfaces::msg::RobotPosition);

    void initialize_ball_filter(oxebots_interfaces::msg::BallPosition msg);
    void initialize_robots_filter();

    void update_ball_filter(oxebots_interfaces::msg::BallPosition msg);
    void update_robots_filter();

    void prediction_ball_filter();
    void prediction_robots_filter();

   private:
    rclcpp::Subscription<oxebots_interfaces::msg::RobotPosition>::SharedPtr
      _robots_subscriber;
    rclcpp::Subscription<oxebots_interfaces::msg::BallPosition>::SharedPtr
      _ball_subscriber;

    rclcpp::Publisher<oxebots_interfaces::msg::BallPredicted>::SharedPtr
      _ball_publisher;
    rclcpp::Publisher<oxebots_interfaces::msg::RobotPredicted>::SharedPtr
      _robots_publisher;

    // Vamos ter um vector de matrizes onde cada matriz pertence ao movimento
    // de um dos robos

    std::vector<RobotControlState> * _allies_state;
    std::vector<RobotCovarianceState> * _allies_covariance;

    std::vector<RobotControlState> * _enemies_state;
    std::vector<RobotCovarianceState> * _enemies_covariance;

    // Assim como ter uma matriz para o movimento da bola
    Eigen::VectorXd _ball_state;
    Eigen::MatrixXd _ball_covariance;

    bool _is_ball_initialized;
    bool _is_robots_initialized;
    unsigned long _initial_timestamp;

    Eigen::MatrixXd _measurement_matrix;
    Eigen::Matrix2d _measurement_noise;
    Eigen::MatrixXd _process_noise;
};
