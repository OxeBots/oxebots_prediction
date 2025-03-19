#include "oxebots_prediction/kalman_filter_node.hpp"

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
    declare_parameter("sensor_measurement_noise", 1.0f);
    declare_parameter("process_noise", 0.8f);

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

    _ball_state = Eigen::VectorXd(8);
    _ball_covariance = Eigen::MatrixXd::Zero(8, 8);

    _is_ball_initialized = false;
    _is_robots_initialized = false;
    _initial_timestamp = std::chrono::system_clock::now().time_since_epoch() /
                         std::chrono::milliseconds(1);

    _measurement_matrix = Eigen::MatrixXd(Eigen::MatrixXd::Zero(2, 8));
    _measurement_matrix(0, 0) = 1;
    _measurement_matrix(1, 1) = 1;

    double sensor_measurment_squared =
      get_parameter("sensor_measurement_noise").as_double() *
      get_parameter("sensor_measurement_noise").as_double();

    _measurement_noise << sensor_measurment_squared, 0,
      sensor_measurment_squared, 0;

    _process_noise = Eigen::MatrixXd::Identity(8, 8) *
                     get_parameter("process_noise").as_double();
}

KalmanFilterNode::~KalmanFilterNode()
{
    delete _allies_covariance;
    delete _allies_state;
    delete _enemies_state;
    delete _enemies_covariance;
}

void KalmanFilterNode::initialize_ball_filter(
  oxebots_interfaces::msg::BallPosition msg)
{
    // Set the initial values
    _ball_state(0) = msg.x;
    _ball_state(1) = msg.y;
    _ball_covariance = Eigen::MatrixXd::Identity(8, 8);
    RCLCPP_INFO(get_logger(), "Initializing the Ball filter...");
}

void KalmanFilterNode::initialize_robots_filter() {}

void KalmanFilterNode::update_ball_filter(
  oxebots_interfaces::msg::BallPosition msg)
{
    RCLCPP_DEBUG(get_logger(), "Updating the ball filter");
    Eigen::VectorXd sensor_data = Eigen::VectorXd(2);
    sensor_data << msg.x, msg.y;

    Eigen::MatrixXd innovation_covariance_matrix =
      _measurement_matrix * _ball_covariance *
        _measurement_matrix.transpose() +
      _measurement_noise;

    Eigen::MatrixXd kalman_gain = _ball_covariance *
                                  _measurement_matrix.transpose() *
                                  innovation_covariance_matrix.inverse();

    Eigen::VectorXd y = sensor_data - _measurement_matrix * _ball_state;
    _ball_state += kalman_gain * y;
    _ball_covariance =
      (Eigen::MatrixXd::Identity(8, 8) * kalman_gain * _measurement_matrix) *
      _ball_covariance;
}

void KalmanFilterNode::update_robots_filter() {}

void KalmanFilterNode::prediction_ball_filter()
{
    int dt = (std::chrono::system_clock::now().time_since_epoch() /
              std::chrono::milliseconds(1)) -
             _initial_timestamp;

    RCLCPP_DEBUG(get_logger(), "Ball prediction");

    // Calculate the f function
    // The velocity is a function by: a_t = a_(t-1) * x + b, and position and
    // velocity are the derivatives. In order by:
    // p_x > p_y > v_x > v_y > a_x > a_y > alpha_x > alpha_y
    Eigen::VectorXd predicted_ball_state = Eigen::VectorXd(8);
    predicted_ball_state << _ball_state(0) + _ball_state(2) * dt +
                              0.5 * _ball_state(4) * dt * dt +
                              (1 / 6.0) * _ball_state(6) * dt * dt * dt,
      _ball_state(1) + _ball_state(3) * dt + 0.5 * _ball_state(5) * dt * dt +
        (1 / 6.0) * _ball_state(7) * dt * dt * dt,
      _ball_state(2) + _ball_state(4) * dt + 0.5 * _ball_state(6) * dt * dt,
      _ball_state(3) + _ball_state(5) * dt + 0.5 * _ball_state(7) * dt * dt,
      _ball_state(4) + _ball_state(6) * dt,
      _ball_state(5) + _ball_state(7) * dt, _ball_state(6), _ball_state(7);

    Eigen::MatrixXd jacobian(8, 8);
    // [1, 0, dt, 0, 0.5 * dt**2, 0, (1/6) * dt**3, 0],
    // [0, 1, 0, dt, 0, 0.5 * dt**2, 0, (1/6) * dt**3],
    // [0, 0, 1, 0, dt, 0, 0.5 * dt**2, 0],
    // [0, 0, 0, 1, 0, dt, 0, 0.5 * dt**2],
    // [0, 0, 0, 0, 1, 0, dt, 0],
    // [0, 0, 0, 0, 0, 1, 0, dt],
    // [0, 0, 0, 0, 0, 0, 1, 0],
    // [0, 0, 0, 0, 0, 0, 0, 1]
    jacobian << 1, 0, dt, 0, 0.5f * dt * dt, 0, (1 / 6.0f) * dt * dt * dt, 0,
      0, 1, 0, dt, 0, 0.5f * dt * dt, 0, (1 / 6.0f) * dt * dt * dt, 0, 0, 1, 0,
      dt, 0, 0.5f * dt * dt, 0, 0, 0, 0, 1, 0, dt, 0, 0.5f * dt * dt, 0, 0, 0,
      0, 1, 0, dt, 0, 0, 0, 0, 0, 0, 1, 0, dt, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 1;

    Eigen::MatrixXd predicted_ball_covariance =
      jacobian * _ball_covariance * jacobian.transpose() + _process_noise;

    // TODO: Send the prediction to the channel
    // RCLCPP_DEBUG(get_logger(), "Publishing ball prediction");
    // _ball_publisher->publish()
}

void KalmanFilterNode::prediction_robots_filter() {}

void KalmanFilterNode::ball_data_callback(
  oxebots_interfaces::msg::BallPosition msg)
{
    if (!_is_ball_initialized)
    {
        _is_ball_initialized = true;
        initialize_ball_filter(msg);
        return;
    }

    update_ball_filter(msg);
    prediction_ball_filter();
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
