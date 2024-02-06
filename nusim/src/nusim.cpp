///
/// \file nusim.cpp
/// \author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// \brief This node simulates the turtlebot in a world
///
/// PUBLISHERS:
///     ~/timestep: [std_msgs/msg/UInt64]                 Current timestep
///     ~/walls:    [visualization_msgs/msg/MarkerArray]  Wall markers
///     ~/obstacles: [visualization_msgs/msg/MarkerArray] Obstacle markers
///
/// SERVICES:
///     ~/reset:    [std_srvs/srv/Empty] Reset the turtlebot
///     ~/teleport: [nusim/srv/Teleport] Teleport the turtlebot
///
/// PARAMETERS:
///     \param rate           (double)   The rate of the simulator
///     \param x0             (double)   Tnitial x value
///     \param y0             (double)   Initial y value
///     \param theta0         (double)   Initial theta value
///     \param arena_x_length (double)   Length of arena in x direction
///     \param arena_y_length (double)   Length of arena in y direction
///     \param obstacles/x    (double[]) X coordinates of obstacles
///     \param obstacles/y    (double[]) Y coordinates of obstacles
///     \param obstacles/r    (double)   Radius of obstacles
///
/// \version 0.1
/// \date 2024-01-22
///
/// \copyright Copyright (c) 2024
///
///
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

#include <std_srvs/srv/empty.hpp>
#include "nusim/srv/teleport.hpp"

#include "turtlelib/diff_drive.hpp"

using rclcpp::QoS;
using rclcpp::Node;
using tf2_ros::TransformBroadcaster;
// using tf2::Quaternion;
// using tf2::Vector3;

/// messages
using rcl_interfaces::msg::ParameterDescriptor;
using std_msgs::msg::UInt64;
using geometry_msgs::msg::TransformStamped;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using nuturtlebot_msgs::msg::WheelCommands;
using nuturtlebot_msgs::msg::SensorData;

/// services
using std_srvs::srv::Empty;
using nusim::srv::Teleport;

/// @brief Simulate the turtlebot in a rviz world.
class NuSim : public Node
{
private:
  /// \brief Timer callback funcrion of the nusim node, calls at every cycle
  void timer_callback__()
  {
    UInt64 msg_timestep;
    msg_timestep.data = timestep__++;
    pub_timestep__->publish(msg_timestep);

    broadcast_tf__();
    update_turtlebot_pos__();
    publish_sensor_data__();
  }

  void update_turtlebot_pos__()
  {
    double left_wheel_speed = wheel_cmd__.left_velocity * motor_cmd_per_rad_sec__;
    double right_wheel_speed = wheel_cmd__.right_velocity * motor_cmd_per_rad_sec__;

    double phi_left_new = turtlebot__.left_wheel() + left_wheel_speed * period__;
    double phi_right_new = turtlebot__.right_wheel() + right_wheel_speed * period__;

    turtlebot__.compute_fk(phi_left_new, phi_right_new);

    turtle_x__ = turtlebot__.config_x();
    turtle_y__ = turtlebot__.config_y();
    turtle_theta__ = turtlebot__.config_theta();
  }

  /// @brief
  void broadcast_tf__()
  {
    TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "nusim/world";
    tf.child_frame_id = "red/base_footprint";

    tf.transform.translation.x = turtle_x__;
    tf.transform.translation.y = turtle_y__;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = sin(turtle_theta__ / 2.0);
    tf.transform.rotation.w = cos(turtle_theta__ / 2.0);

    tf_broadcaster__->sendTransform(tf);
  }

  void publish_sensor_data__()
  {
    SensorData msg_sensor;

    msg_sensor.stamp = this->get_clock()->now();
    msg_sensor.left_encoder = (int32_t) (turtlebot__.left_wheel() * encoder_ticks_per_rad__);
    msg_sensor.right_encoder = (int32_t) (turtlebot__.right_wheel() * encoder_ticks_per_rad__);

    pub_sensor_data__->publish(msg_sensor);
  }

  /// \brief publish the markers to display wall on rviz
  void publish_wall_markers__()
  {
    Marker m1;
    Marker m2;
    Marker m3;
    Marker m4;
    MarkerArray m_array;

    /// first wall
    m1.header.stamp = this->get_clock()->now();
    m1.header.frame_id = "nusim/world";
    m1.id = 1;
    m1.type = Marker::CUBE;
    m1.action = Marker::ADD;
    m1.pose.position.x = arena_x_length__ / 2.0 + wall_thickness__ / 2.0;
    m1.pose.position.y = wall_thickness__ / 2.0;
    m1.pose.position.z = wall_height__ / 2.0;
    m1.scale.x = wall_thickness__;
    m1.scale.y = arena_y_length__ + wall_thickness__;
    m1.scale.z = wall_height__;
    m1.color.r = wall_r__;
    m1.color.g = wall_g__;
    m1.color.b = wall_b__;
    m1.color.a = 1.0;

    /// second wall
    m2.header.stamp = this->get_clock()->now();
    m2.header.frame_id = "nusim/world";
    m2.id = 2;
    m2.type = Marker::CUBE;
    m2.action = Marker::ADD;
    m2.pose.position.x = -wall_thickness__ / 2.0;
    m2.pose.position.y = arena_y_length__ / 2.0 + wall_thickness__ / 2.0;
    m2.pose.position.z = wall_height__ / 2.0;
    m2.scale.x = arena_x_length__ + wall_thickness__;
    m2.scale.y = wall_thickness__;
    m2.scale.z = wall_height__;
    m2.color.r = wall_r__;
    m2.color.g = wall_g__;
    m2.color.b = wall_b__;
    m2.color.a = 1.0;

    /// third wall
    m3.header.stamp = this->get_clock()->now();
    m3.header.frame_id = "nusim/world";
    m3.id = 3;
    m3.type = Marker::CUBE;
    m3.action = Marker::ADD;
    m3.pose.position.x = -arena_x_length__ / 2.0 - wall_thickness__ / 2.0;
    m3.pose.position.y = -wall_thickness__ / 2.0;
    m3.pose.position.z = wall_height__ / 2.0;
    m3.scale.x = wall_thickness__;
    m3.scale.y = arena_y_length__ + wall_thickness__;
    m3.scale.z = wall_height__;
    m3.color.r = wall_r__;
    m3.color.g = wall_g__;
    m3.color.b = wall_b__;
    m3.color.a = 1.0;

    /// fourth wall
    m4.header.stamp = this->get_clock()->now();
    m4.header.frame_id = "nusim/world";
    m4.id = 4;
    m4.type = Marker::CUBE;
    m4.action = Marker::ADD;
    m4.pose.position.x = wall_thickness__ / 2.0;
    m4.pose.position.y = -arena_y_length__ / 2.0 - wall_thickness__ / 2.0;
    m4.pose.position.z = wall_height__ / 2.0;
    m4.scale.x = arena_x_length__ + wall_thickness__;
    m4.scale.y = wall_thickness__;
    m4.scale.z = wall_height__;
    m4.color.r = wall_r__;
    m4.color.g = wall_g__;
    m4.color.b = wall_b__;
    m4.color.a = 1.0;

    m_array.markers.push_back(m1);
    m_array.markers.push_back(m2);
    m_array.markers.push_back(m3);
    m_array.markers.push_back(m4);

    pub_wall_markers__->publish(m_array);
  }

  /// \brief publish marker to display obstacle on rviz
  void publish_obstacle_markers__()
  {
    MarkerArray m_array_obs;

    for (std::size_t i = 0; i < obstacles_x__.size(); i++) {
      Marker m_obs;

      m_obs.header.stamp = this->get_clock()->now();
      m_obs.header.frame_id = "nusim/world";
      m_obs.id = i + 10;
      m_obs.type = Marker::CYLINDER;
      m_obs.action = Marker::ADD;
      m_obs.pose.position.x = obstacles_x__.at(i);
      m_obs.pose.position.y = obstacles_y__.at(i);
      m_obs.pose.position.z = obstacle_height__ / 2.0;
      m_obs.scale.x = 2.0 * obstacle_radius__;
      m_obs.scale.y = 2.0 * obstacle_radius__;
      m_obs.scale.z = obstacle_height__;
      m_obs.color.r = 1.0;
      m_obs.color.g = 0.0;
      m_obs.color.b = 0.0;
      m_obs.color.a = 1.0;

      m_array_obs.markers.push_back(m_obs);
    }

    pub_obstacle_markers__->publish(m_array_obs);
  }

  /// \brief reset the position of the turtlebot.
  void reset_turtle_pose__()
  {
    period__ = 1.0 / rate__;
    turtle_x__ = x_0__;
    turtle_y__ = y_0__;
    turtle_theta__ = theta_0__;
    turtlebot__ = turtlelib::DiffDrive(track_width__, wheel_radius__);
  }

  /// \brief Callback function for reset service, reset the position of turtlebot and timestep
  /// \param request The request object
  /// \param response The response object
  void srv_reset_callback__(
    std::shared_ptr<Empty::Request> request,
    std::shared_ptr<Empty::Response> response)
  {
    (void) request;
    (void) response;

    reset_turtle_pose__();
    timestep__ = 0;
  }

  /// \brief Callback function of the teleport service, teleport the turtlebot to a place
  /// \param request The request object
  /// \param response The response object
  void srv_teleport_callback__(
    std::shared_ptr<Teleport::Request> request,
    std::shared_ptr<Teleport::Response> response)
  {
    turtle_x__ = request->x;
    turtle_y__ = request->y;
    turtle_theta__ = request->theta;

    response->result = true;
  }

  void sub_wheel_cmd_callback__(WheelCommands::SharedPtr msg)
  {
    wheel_cmd__ = *msg;
  }

  /// timer
  rclcpp::TimerBase::SharedPtr timer__;

  /// services
  rclcpp::Service<Empty>::SharedPtr srv_reset__;
  rclcpp::Service<Teleport>::SharedPtr srv_teleport__;

  /// subscribers
  rclcpp::Subscription<WheelCommands>::SharedPtr sub_wheel_cmd__;

  /// publishers
  rclcpp::Publisher<UInt64>::SharedPtr pub_timestep__;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_wall_markers__;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_obstacle_markers__;
  rclcpp::Publisher<SensorData>::SharedPtr pub_sensor_data__;

  /// transform broadcasters
  std::unique_ptr<TransformBroadcaster> tf_broadcaster__;

  /// qos profile
  rclcpp::QoS marker_qos__;

  /// subscribed messages
  WheelCommands wheel_cmd__;

  /// parameters
  double rate__;
  double x_0__;
  double y_0__;
  double theta_0__;
  double arena_x_length__;
  double arena_y_length__;
  std::vector<double> obstacles_x__;
  std::vector<double> obstacles_y__;
  double obstacle_radius__;
  double wheel_radius__;
  double track_width__;
  int64_t motor_cmd_max__;
  double motor_cmd_per_rad_sec__;
  double encoder_ticks_per_rad__;

  /// other attributes
  double period__;
  uint64_t timestep__;
  double turtle_x__;
  double turtle_y__;
  double turtle_theta__;
  double wall_r__;
  double wall_g__;
  double wall_b__;
  double wall_height__;
  double wall_thickness__;
  double obstacle_height__;
  turtlelib::DiffDrive turtlebot__;

public:
  /// \brief Initialize the nusim node
  NuSim()
  : Node("nusim"), marker_qos__(10), timestep__(0), wall_r__(1.0), wall_g__(0.0), wall_b__(0.0),
    wall_height__(0.25), wall_thickness__(0.1), obstacle_height__(0.25)
  {
    /// parameter descriptions
    ParameterDescriptor rate_des;
    ParameterDescriptor x0_des;
    ParameterDescriptor y0_des;
    ParameterDescriptor theta0_des;
    ParameterDescriptor arena_x_des;
    ParameterDescriptor arena_y_des;
    ParameterDescriptor obs_x_des;
    ParameterDescriptor obs_y_des;
    ParameterDescriptor obs_r_des;
    ParameterDescriptor wheel_radius_des;
    ParameterDescriptor track_width_des;
    ParameterDescriptor motor_cmd_max_des;
    ParameterDescriptor motor_cmd_per_rad_sec_des;
    ParameterDescriptor encoder_ticks_per_rad_des;
    rate_des.description = "The rate of the simulator";
    x0_des.description = "The initial x location";
    y0_des.description = "The initial y location";
    theta0_des.description = "The initial theta pose";
    arena_x_des.description = "The wall length in x direction";
    arena_y_des.description = "The wall length in y direction";
    obs_x_des.description = "The list of x coordinates of the obstacles";
    obs_y_des.description = "The list of y coordinates of the obstacles";
    obs_r_des.description = "The radius of the obstacles";
    wheel_radius_des.description = "The radius of the wheel";
    track_width_des.description = "The width of the track";
    motor_cmd_max_des.description = "The maximum motor command";
    motor_cmd_per_rad_sec_des.description = "The motor command per rad/s speed";
    encoder_ticks_per_rad_des.description = "Number of encoder ticks per radian";

    /// declare parameters
    this->declare_parameter<double>("rate", 100.0, rate_des);
    this->declare_parameter<double>("x0", 0.0, x0_des);
    this->declare_parameter<double>("y0", 0.0, y0_des);
    this->declare_parameter<double>("theta0", 0.0, theta0_des);
    this->declare_parameter<double>("arena_x_length", 10.0, arena_x_des);
    this->declare_parameter<double>("arena_y_length", 10.0, arena_y_des);
    this->declare_parameter<std::vector<double>>(
      "obstacles/x",
      std::vector<double>{1.2, 2.3},
      obs_x_des
    );
    this->declare_parameter<std::vector<double>>(
      "obstacles/y",
      std::vector<double>{2.3, 4.5},
      obs_y_des
    );
    this->declare_parameter<double>("obstacles/r", 0.05, obs_r_des);
    this->declare_parameter<double>("wheel_radius", 0.033, wheel_radius_des);
    this->declare_parameter<double>("track_width", 0.16, track_width_des);
    this->declare_parameter<int64_t>("motor_cmd_max", 265, motor_cmd_max_des);
    this->declare_parameter<double>("motor_cmd_per_rad_sec", 0.024, motor_cmd_max_des);
    this->declare_parameter<double>("encoder_ticks_per_rad", 651.9, encoder_ticks_per_rad_des);

    /// get parameter values
    rate__ = this->get_parameter("rate").as_double();
    x_0__ = this->get_parameter("x0").as_double();
    y_0__ = this->get_parameter("y0").as_double();
    theta_0__ = this->get_parameter("theta0").as_double();
    arena_x_length__ = this->get_parameter("arena_x_length").as_double();
    arena_y_length__ = this->get_parameter("arena_y_length").as_double();
    obstacles_x__ = this->get_parameter("obstacles/x").as_double_array();
    obstacles_y__ = this->get_parameter("obstacles/y").as_double_array();
    obstacle_radius__ = this->get_parameter("obstacles/r").as_double();
    wheel_radius__ = this->get_parameter("wheel_radius").as_double();
    track_width__ = this->get_parameter("track_width").as_double();
    motor_cmd_max__ = this->get_parameter("motor_cmd_max").as_int();
    motor_cmd_per_rad_sec__ = this->get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad__ = this->get_parameter("encoder_ticks_per_rad").as_double();

    /// check for x y length
    if (obstacles_x__.size() != obstacles_y__.size()) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "The x and y coordinates should have the same length"
      );
      throw std::invalid_argument("The x and y coordinate should have same length");
      exit(EXIT_FAILURE);
    }

    /// initialize attributes
    reset_turtle_pose__();

    // set marker qos policy
    marker_qos__.transient_local();

    /// timer
    timer__ = this->create_wall_timer(
      std::chrono::duration<long double>{period__},
      std::bind(&NuSim::timer_callback__, this));

    /// services
    srv_reset__ = this->create_service<Empty>(
      "~/reset",
      std::bind(
        &NuSim::srv_reset_callback__, this, std::placeholders::_1,
        std::placeholders::_2));
    srv_teleport__ =
      this->create_service<Teleport>(
      "~/teleport",
      std::bind(
        &NuSim::srv_teleport_callback__, this, std::placeholders::_1,
        std::placeholders::_2));

    // subscribers
    sub_wheel_cmd__ =
      this->create_subscription<WheelCommands>(
      "red/wheel_cmd", 10,
      std::bind(&NuSim::sub_wheel_cmd_callback__, this, std::placeholders::_1));

    // publishers
    pub_timestep__ = this->create_publisher<UInt64>("~/timestep", 10);
    pub_sensor_data__ = this->create_publisher<SensorData>("red/sensor_data", 10);
    pub_wall_markers__ = this->create_publisher<MarkerArray>("~/walls", marker_qos__);
    pub_obstacle_markers__ = this->create_publisher<MarkerArray>("~/obstacles", marker_qos__);

    /// transform broadcasters
    tf_broadcaster__ = std::make_unique<TransformBroadcaster>(*this);

    publish_wall_markers__();
    publish_obstacle_markers__();
  }
};

/// \brief The main function for the nusim node
/// \param argc number of arguments
/// \param argv value of the arguments
/// \return result code
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_nusim = std::make_shared<NuSim>();
  rclcpp::spin(node_nusim);

  rclcpp::shutdown();
  return 0;
}
