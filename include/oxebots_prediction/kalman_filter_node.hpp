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
#include "oxebots_interfaces/msg/ball_prediction.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"
#include "oxebots_interfaces/msg/robot_prediction.hpp"

typedef struct RobotControlState
{
    Eigen::VectorXd state;
    uint32_t id;
    bool initialized;
} RobotControlState;

typedef struct RobotCovarianceState
{
    Eigen::MatrixXd covariance;
    uint32_t id;
} RobotCovarianceState;

class KalmanFilterNode : public rclcpp::Node
{
   public:
    KalmanFilterNode();
    ~KalmanFilterNode();

   protected:
    void ball_data_callback(oxebots_interfaces::msg::BallPosition msg);
    void robots_data_callback(oxebots_interfaces::msg::RobotPosition msg);

    void initialize_ball_filter(oxebots_interfaces::msg::BallPosition msg);
    
    void update_ball_filter(oxebots_interfaces::msg::BallPosition msg);
    void update_robots_filter(oxebots_interfaces::msg::RobotPosition msg);

    void prediction_ball_filter();
    void prediction_robots_filter();

   private:
    rclcpp::Subscription<oxebots_interfaces::msg::RobotPosition>::SharedPtr
      _robots_subscriber;
    rclcpp::Subscription<oxebots_interfaces::msg::BallPosition>::SharedPtr
      _ball_subscriber;

    rclcpp::Publisher<oxebots_interfaces::msg::BallPrediction>::SharedPtr
      _ball_publisher;
    rclcpp::Publisher<oxebots_interfaces::msg::RobotPrediction>::SharedPtr
      _robots_publisher;

    std::vector<RobotControlState> _allies_state;
    std::vector<RobotCovarianceState> _allies_covariance;

    std::vector<RobotControlState> _enemies_state;
    std::vector<RobotCovarianceState> _enemies_covariance;

    // Assim como ter uma matriz para o movimento da bola
    Eigen::VectorXd _ball_state;
    Eigen::MatrixXd _ball_covariance;

    bool _is_ball_initialized;
    bool _is_robots_initialized;
    
    rclcpp::Time _last_ball_time;
    rclcpp::Time _last_robots_time;

    Eigen::MatrixXd _measurement_matrix;
    Eigen::Matrix2d _measurement_noise;
    Eigen::MatrixXd _process_noise;
};
