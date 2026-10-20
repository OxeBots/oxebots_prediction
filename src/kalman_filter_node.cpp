#include "oxebots_prediction/kalman_filter_node.hpp"

KalmanFilterNode::KalmanFilterNode() : rclcpp::Node("oxebots_kalman_filter")
{
    declare_parameter("ball_publisher", "ball_predicted");
    declare_parameter("ball_subscriber", "ball_data");
    declare_parameter("robot_publisher", "robot_predicted");
    declare_parameter("robot_subscriber", "robot_data");
    declare_parameter("maximum_publisher_size", 10);
    declare_parameter("team_size", 3);

    // Parâmetros do filtro da bola
    declare_parameter("ball_pos_measurement_noise", 3.0);
    declare_parameter("ball_process_noise", 2000.0);
    declare_parameter("ball_friction_accel", 800.0);
    declare_parameter("ball_kick_threshold_mm", 80.0);

    // Parâmetros do filtro do robô
    declare_parameter("robot_pos_measurement_noise", 3.0);
    declare_parameter("robot_angle_measurement_noise", 0.02);
    declare_parameter("robot_pos_process_noise", 4000.0);
    declare_parameter("robot_angle_process_noise", 30.0);

    // Carregar configurações da bola
    _ball_config.pos_measurement_noise = get_parameter("ball_pos_measurement_noise").as_double();
    _ball_config.process_noise = get_parameter("ball_process_noise").as_double();
    _ball_config.friction_accel = get_parameter("ball_friction_accel").as_double();
    _ball_config.kick_threshold_mm = get_parameter("ball_kick_threshold_mm").as_double();
    _ball_filter = oxebots_prediction::BallKalmanFilter(_ball_config);

    // Carregar configurações dos robôs
    _robot_config.pos_measurement_noise = get_parameter("robot_pos_measurement_noise").as_double();
    _robot_config.angle_measurement_noise = get_parameter("robot_angle_measurement_noise").as_double();
    _robot_config.pos_process_noise = get_parameter("robot_pos_process_noise").as_double();
    _robot_config.angle_process_noise = get_parameter("robot_angle_process_noise").as_double();

    int max_pub_size = get_parameter("maximum_publisher_size").as_int();

    _ball_publisher = create_publisher<oxebots_interfaces::msg::BallPrediction>(
        get_parameter("ball_publisher").as_string(), max_pub_size);

    _robots_publisher = create_publisher<oxebots_interfaces::msg::RobotPrediction>(
        get_parameter("robot_publisher").as_string(), max_pub_size);

    _ball_subscriber = create_subscription<oxebots_interfaces::msg::BallPosition>(
        get_parameter("ball_subscriber").as_string(), max_pub_size,
        std::bind(&KalmanFilterNode::ball_data_callback, this, std::placeholders::_1));

    _robots_subscriber = create_subscription<oxebots_interfaces::msg::RobotPosition>(
        get_parameter("robot_subscriber").as_string(), max_pub_size,
        std::bind(&KalmanFilterNode::robots_data_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Filtro de Kalman iniciado com modelos dedicados (Ball 4D + Atrito / Robot 6D).");
}

void KalmanFilterNode::ball_data_callback(const oxebots_interfaces::msg::BallPosition::SharedPtr msg)
{
    rclcpp::Time current_time = now();

    if (!_is_ball_initialized)
    {
        _is_ball_initialized = true;
        _ball_filter.init(msg->x, msg->y);
        _last_ball_time = current_time;
        RCLCPP_INFO(get_logger(), "Filtro da bola inicializado em (%.1f, %.1f).", msg->x, msg->y);
        return;
    }

    double dt = (current_time - _last_ball_time).seconds();
    _last_ball_time = current_time;

    // Reset se houve pausa longa ou descontinuidade temporal
    if (dt > 1.0) {
        _ball_filter.init(msg->x, msg->y);
        dt = 0.0;
    }

    // 1. Predição canônica com atrito de rolamento no carpete
    if (dt > 0.0) {
        _ball_filter.predict(dt);
    }

    // 2. Atualização com medição da visão
    _ball_filter.update(msg->x, msg->y, dt);

    // 3. Publicação dos estados preditos
    oxebots_interfaces::msg::BallPrediction prediction_msg;
    prediction_msg.x = _ball_filter.x();
    prediction_msg.y = _ball_filter.y();
    prediction_msg.vx = _ball_filter.vx();
    prediction_msg.vy = _ball_filter.vy();
    prediction_msg.ax = _ball_filter.ax();
    prediction_msg.ay = _ball_filter.ay();

    _ball_publisher->publish(prediction_msg);
}

void KalmanFilterNode::robots_data_callback(const oxebots_interfaces::msg::RobotPosition::SharedPtr msg)
{
    rclcpp::Time current_time = now();

    if (!_is_robots_initialized)
    {
        _is_robots_initialized = true;
        _last_robots_time = current_time;
        RCLCPP_INFO(get_logger(), "Filtro dos robôs inicializado.");
    }

    double dt = (current_time - _last_robots_time).seconds();
    _last_robots_time = current_time;

    if (dt > 1.0) {
        dt = 0.0;
    }

    auto process_robots = [&](const std::vector<oxebots_interfaces::msg::RobotGameData> & robots,
                              std::map<uint32_t, oxebots_prediction::RobotKalmanFilter> & filters)
        -> std::vector<oxebots_interfaces::msg::RobotPredictionData>
    {
        std::vector<oxebots_interfaces::msg::RobotPredictionData> result;
        result.reserve(robots.size());

        for (const auto & robot : robots) {
            auto it = filters.find(robot.id);
            if (it == filters.end()) {
                oxebots_prediction::RobotKalmanFilter filter(_robot_config);
                filter.init(robot.x, robot.y, robot.orientation);
                filters[robot.id] = filter;
            } else {
                if (dt > 0.0) {
                    it->second.predict(dt);
                }
                it->second.update(robot.x, robot.y, robot.orientation);
            }

            const auto & f = filters[robot.id];
            oxebots_interfaces::msg::RobotPredictionData data;
            data.id = robot.id;
            data.x = f.x();
            data.y = f.y();
            data.vx = f.vx();
            data.vy = f.vy();
            data.ax = 0.0;
            data.ay = 0.0;
            data.orientation = f.orientation();
            data.vorientation = f.angular_velocity();
            result.push_back(data);
        }
        return result;
    };

    oxebots_interfaces::msg::RobotPrediction predicted_msg;
    predicted_msg.allies = process_robots(msg->allies, _allies_filters);
    predicted_msg.enemies = process_robots(msg->enemies, _enemies_filters);

    _robots_publisher->publish(predicted_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
