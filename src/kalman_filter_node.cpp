#include "oxebots_prediction/kalman_filter_node.hpp"

KalmanFilterNode::KalmanFilterNode() : rclcpp::Node("oxebots_kalman_filter") {}

KalmanFilterNode::~KalmanFilterNode() {}

void KalmanFilterNode::initialize_ball_filter() {}

void KalmanFilterNode::initialize_robots_filter() {}

void KalmanFilterNode::update_ball_filter() {}

void KalmanFilterNode::update_robots_filter() {}

void KalmanFilterNode::prediction_ball_filter() {}

void KalmanFilterNode::prediction_robots_filter() {}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
