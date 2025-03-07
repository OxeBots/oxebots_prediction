#include "oxebots_prediction/kalman_filter_node.hpp"

KalmanFilterNode::KalmanFilterNode() : rclcpp::Node("oxebots_kalman_filter") {}

KalmanFilterNode::~KalmanFilterNode() {}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
