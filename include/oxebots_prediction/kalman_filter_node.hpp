#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <map>
#include <memory>
#include <vector>

#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/ball_prediction.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"
#include "oxebots_interfaces/msg/robot_prediction.hpp"

#include "oxebots_prediction/ball_kalman_filter.hpp"
#include "oxebots_prediction/robot_kalman_filter.hpp"

class KalmanFilterNode : public rclcpp::Node
{
public:
    KalmanFilterNode();
    ~KalmanFilterNode() override = default;

private:
    void ball_data_callback(const oxebots_interfaces::msg::BallPosition::SharedPtr msg);
    void robots_data_callback(const oxebots_interfaces::msg::RobotPosition::SharedPtr msg);

    rclcpp::Subscription<oxebots_interfaces::msg::RobotPosition>::SharedPtr _robots_subscriber;
    rclcpp::Subscription<oxebots_interfaces::msg::BallPosition>::SharedPtr _ball_subscriber;

    rclcpp::Publisher<oxebots_interfaces::msg::BallPrediction>::SharedPtr _ball_publisher;
    rclcpp::Publisher<oxebots_interfaces::msg::RobotPrediction>::SharedPtr _robots_publisher;

    oxebots_prediction::BallFilterConfig _ball_config;
    oxebots_prediction::RobotFilterConfig _robot_config;

    oxebots_prediction::BallKalmanFilter _ball_filter;
    std::map<uint32_t, oxebots_prediction::RobotKalmanFilter> _allies_filters;
    std::map<uint32_t, oxebots_prediction::RobotKalmanFilter> _enemies_filters;

    bool _is_ball_initialized{false};
    bool _is_robots_initialized{false};

    rclcpp::Time _last_ball_time;
    rclcpp::Time _last_robots_time;
};
