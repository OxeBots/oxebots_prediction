#pragma once
#include <boost/optional.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"

class KalmanFilterNode : public rclcpp::Node
{
   public:
    KalmanFilterNode();
    ~KalmanFilterNode();

   private:
    rclcpp::Subscription<oxebots_interfaces::msg::RobotPosition>::SharedPtr
      robots_subscriber;
    rclcpp::Subscription<oxebots_interfaces::msg::BallPosition>::SharedPtr ball_subscriber;
};
