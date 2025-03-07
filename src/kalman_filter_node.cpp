#include "oxebots_prediction/kalman_filter_node.hpp"

#include <functional>

#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"

KalmanFilterNode::KalmanFilterNode() : rclcpp::Node("oxebots_kalman_filter")
{
    declare_parameter("ball_publisher", "ball_publisher");
    declare_parameter("ball_subscriber", "ball_subscriber");
    declare_parameter("robot_publisher", "robot_publisher");
    declare_parameter("robot_subscriber", "robot_subscriber");
    declare_parameter("maximum_publisher_size", 10);

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
