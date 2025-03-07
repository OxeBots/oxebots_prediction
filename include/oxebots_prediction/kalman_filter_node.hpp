#pragma once
#include <sys/types.h>

#include <boost/optional.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <Eigen/Dense>
#include <cstdint>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/ball_predicted.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"
#include "oxebots_interfaces/msg/robot_predicted.hpp"

typedef struct RobotControlState
{
    Eigen::Vector4d state;
    uint32_t id;
} RobotControlState;

typedef struct RobotCovarianceState
{
    Eigen::Matrix4d covariance;
    uint32_t id;
} RobotCovarianceState;

class KalmanFilterNode : public rclcpp::Node
{
   public:
    KalmanFilterNode();
    ~KalmanFilterNode();

   protected:
    void initialize_ball_filter();
    void initialize_robots_filter();

    void update_ball_filter();
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

    std::vector<RobotControlState> _allies_state;
    std::vector<RobotCovarianceState> _allies_covariance;

    std::vector<RobotControlState> _enemies_state;
    std::vector<RobotCovarianceState> _enemies_covariance;

    // Assim como ter uma matriz para o movimento da bola
    Eigen::Vector4d _ball_state;
    Eigen::Matrix4d _ball_covariance;
};
