/// \file slam.cpp
/// \author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// \brief Update the odometry of a robot using SLAM algorithm.
///
/// PARAMETERS:
///   \param body_id                [string]  The id of the body frame
///   \param odom_id                [string]  The id of the odom frame
///   \param wheel_left             [string]  The left wheel joint name
///   \param wheel_right            [string]  The right wheel joint name
///   \param wheel_radius           [double]  The radius of ythe wheel
///   \param track_width            [double]  The distance between two wheels
///   \param input_noice            [double]  The input noice
///   \param basic_sensor_variance  [double]  The variance of the sensor
///
/// SUBSCRIPTIONS:
///   joint_states  [sensor_msgs/msg/JointState]                    The joint state of the robot.
///   obs_pos       [nuturtle_interfaces/msg/ObstacleMeasurements]  The measurements of the obstacles
///
/// PUBLISHERS:
///   odom          [nav_msgs/msg/Odomoetry]                        The odometry of the node.
///   path          [nav_msgs//msgPath]                             The path robot follows.
///   ~/map         [visualization/msg/MarkerArray]                 The mapped obstacle markers.
///
/// SERVICES:
///   initial_pose [nuturtle_interfaces/srv/InitialPose]            Reset the initial pose.
///
/// \version 0.1
/// \date 2024-02-15
///
/// \copyright Copyright (c) 2024
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/qos.hpp>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "nuturtle_interfaces/msg/obstacle_measurements.hpp"

#include "nuturtle_interfaces/srv/initial_pose.hpp"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf_slam.hpp"

using namespace std::chrono_literals;

using rclcpp::Node;
using tf2_ros::TransformBroadcaster;

using rcl_interfaces::msg::ParameterDescriptor;
using sensor_msgs::msg::JointState;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::PoseStamped;
using nuturtle_interfaces::msg::ObstacleMeasurements;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using nuturtle_interfaces::srv::InitialPose;

/// \brief The slam algorithm based on Extended Kalman Filter
class Slam : public Node
{
private:
  /// \brief The timer callback of the odometry node
  void timer_callback_()
  {
    if (joint_states_available_) {
      broadcast_tf_();
      publish_odom_();
      publish_path_();
    }
  }

  /// \brief Update the turtlebot configuration based on new joint states,
  ///        then publish new odom
  /// \param msg The subscibed joint_states message.
  void sub_joint_states_callback_(JointState::SharedPtr msg)
  {
    if (!joint_states_available_) {
      joint_states_available_ = true;
      joint_states_curr_.header.stamp = msg->header.stamp;
      joint_states_curr_.name = msg->name;
      joint_states_curr_.position = msg->position;

      for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name.at(i) == wheel_left_) {
          index_left_ = i;
          left_init_ = msg->position.at(i);
        } else if (msg->name.at(i) == wheel_right_) {
          index_right_ = i;
          right_init_ = msg->position.at(i);
        }
      }

      if (index_left_ == INT64_MAX || index_right_ == INT64_MAX) {
        RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel joint name");
        throw std::logic_error("Invalid wheel joint name");
      }

      turtlebot_.update_wheel(left_init_, right_init_);
    }

    joint_states_prev_ = joint_states_curr_;
    joint_states_curr_.header.stamp = msg->header.stamp;
    joint_states_curr_.name = msg->name;
    joint_states_curr_.position = msg->position;
  }

  /// \brief The callback function of the obstacle measurement
  /// \param msg The subcribed message object
  void sub_obs_measure_callback_(ObstacleMeasurements::SharedPtr msg)
  {
    obs_measure_ = *msg;

    const turtlelib::Transform2D Tob(
      {turtlebot_.config_x(), turtlebot_.config_y()},
      turtlebot_.config_theta()
    );
    const auto Tmb = Tmo_ * Tob;
    RCLCPP_DEBUG_STREAM(get_logger(), "Robot Position: " << Tmb);

    double theta_est = Tmb.rotation();
    double x_est = Tmb.translation().x;
    double y_est = Tmb.translation().y;

    const auto state_prev = turtle_slam_.get_robot_state();

    const auto A_mat = turtle_slam_.get_A_mat(x_est - state_prev.x, y_est - state_prev.y);
    RCLCPP_DEBUG_STREAM(get_logger(), "A_mat: " << std::endl << A_mat);

    const auto Sigma_pre = turtle_slam_.get_covariance_mat();
    const auto Sigma_est = A_mat * Sigma_pre * A_mat.t() + Q_mat_;
    RCLCPP_DEBUG_STREAM(get_logger(), "Sigma_mat: " << std::endl << Sigma_est);

    arma::vec state_curr = turtle_slam_.get_state_vec();

    state_curr.at(0) = theta_est;
    state_curr.at(1) = x_est;
    state_curr.at(2) = y_est;

    arma::mat Sigma_curr(Sigma_est);
    RCLCPP_DEBUG_STREAM(get_logger(), "State: " << std::endl << state_curr);

    for (size_t i = 0; i < msg->measurements.size(); ++i) {
      const auto measure = msg->measurements.at(i);
      const auto uid = measure.uid;
      const auto x = measure.x;
      const auto y = measure.y;
      RCLCPP_DEBUG_STREAM(get_logger(), "uid: " << uid);
      theta_est = state_curr.at(0);
      x_est = state_curr.at(1);
      y_est = state_curr.at(2);

      const arma::vec z_vec = turtle_slam_.get_h_vec({x, y, uid});
      RCLCPP_DEBUG_STREAM(get_logger(), "z_vec: " << std::endl << z_vec);

      auto landmark_pos = turtle_slam_.get_landmark_pos(uid);
      if (landmark_pos.uid == -1) {
        landmark_pos.x = x_est + z_vec.at(0) * cos(theta_est + z_vec.at(1));
        landmark_pos.y = y_est + z_vec.at(0) * sin(theta_est + z_vec.at(1));

        state_curr(3 + 2 * uid) = landmark_pos.x;
        state_curr(3 + 2 * uid + 1) = landmark_pos.y;
      }

      turtlelib::Point2D pm_land{landmark_pos.x, landmark_pos.y};
      turtlelib::Transform2D Tmb_land({x_est, y_est}, theta_est);
      turtlelib::Transform2D Tbm_land = Tmb_land.inv();
      turtlelib::Point2D pb_land = Tbm_land(pm_land);
      const auto dx = pb_land.x;
      const auto dy = pb_land.y;
      RCLCPP_DEBUG_STREAM(get_logger(), "x: " << dx << ", y: " << dy);

      turtlelib::Measurement est_body = {dx, dy, uid};
      turtlelib::Measurement est_world =
      {
        landmark_pos.x - x_est,
        landmark_pos.y - y_est,
        uid
      };

      const arma::vec z_hat = turtle_slam_.get_h_vec(est_body);  // + dist_sensor_(generator_);
      RCLCPP_DEBUG_STREAM(get_logger(), "z_hat: " << std::endl << z_hat);

      const arma::mat H_mat = turtle_slam_.get_H_mat(est_world, uid);
      RCLCPP_DEBUG_STREAM(get_logger(), "H_mat: " << std::endl << H_mat);

      const arma::mat K_mat = Sigma_curr * H_mat.t() *
        (H_mat * Sigma_curr * H_mat.t() + sensor_noice_).i();
      RCLCPP_DEBUG_STREAM(get_logger(), "K_mat: " << std::endl << K_mat);

      arma::vec dz_vec = z_vec - z_hat;
      dz_vec.at(1) = turtlelib::normalize_angle(dz_vec.at(1));
      RCLCPP_DEBUG_STREAM(get_logger(), "dz_vec: " << std::endl << dz_vec);

      arma::vec update = K_mat * dz_vec;
      update.at(0) = turtlelib::normalize_angle(update.at(0));
      RCLCPP_DEBUG_STREAM(get_logger(), "Update: " << std::endl << update);

      const arma::mat I_mat(2 * num_obstacles_ + 3, 2 * num_obstacles_ + 3, arma::fill::eye);

      state_curr += update;
      state_curr.at(0) = turtlelib::normalize_angle(state_curr.at(0));
      Sigma_curr = (I_mat - (K_mat * H_mat)) * Sigma_curr;
      RCLCPP_DEBUG_STREAM(get_logger(), "State vec: " << std::endl << state_curr);
      turtle_slam_.update_landmark_pos(state_curr);
    }

    const auto theta_new = state_curr.at(0);
    const auto x_new = state_curr.at(1);
    const auto y_new = state_curr.at(2);


    update_map_odom_tf_(x_new, y_new, theta_new);
    publish_map_markers();

    turtle_slam_.update_covariance(Sigma_curr);
    turtle_slam_.update_state(x_new, y_new, theta_new);
  }

  /// \brief The initial pose service callback function
  /// \param request The initial pose service request
  /// \param respose The initial pose service response
  void srv_initial_pose_callback_(
    std::shared_ptr<InitialPose::Request> request,
    std::shared_ptr<InitialPose::Response> respose
  )
  {
    double x = request->x;
    double y = request->y;
    double theta = request->theta;

    turtlebot_.update_config(x, y, theta);

    respose->success = true;
  }

  /// \brief Publish markers for mapped obstacles
  void publish_map_markers()
  {
    MarkerArray map_array_msg;
    std::vector<turtlelib::Measurement> landmarks = turtle_slam_.get_all_landmarks();

    for (size_t i = 0; i < landmarks.size(); ++i) {
      const auto x = landmarks.at(i).x;
      const auto y = landmarks.at(i).y;

      if (
        !turtlelib::almost_equal(x, 1e4) &&
        !turtlelib::almost_equal(y, 1e4))
      {
        Marker m;

        m.header.stamp = get_clock()->now();
        m.header.frame_id = map_id_;
        m.id = 30 + i;
        m.type = Marker::CYLINDER;
        m.action = Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = marker_height_ / 2.0;
        m.scale.x = 2.0 * marker_radius_;
        m.scale.y = 2.0 * marker_radius_;
        m.scale.z = marker_height_;
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        m.color.a = 1.0;

        map_array_msg.markers.push_back(m);
      }
    }

    pub_map_array_->publish(map_array_msg);
  }

  /// \brief Publish the path that green robot follows
  void publish_path_()
  {
    const auto x_turtle = turtlebot_.config_x();
    const auto y_turtle = turtlebot_.config_y();
    const auto theta_turtle = turtlebot_.config_theta();

    turtlelib::Transform2D Tob{{x_turtle, y_turtle}, theta_turtle};
    turtlelib::Transform2D Tmb = Tmo_ * Tob;

    Path msg_path;

    msg_path.header.stamp = get_clock()->now();
    msg_path.header.frame_id = map_id_;

    PoseStamped pose;

    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = map_id_;

    pose.pose.position.x = Tmb.translation().x;
    pose.pose.position.y = Tmb.translation().y;
    pose.pose.position.z = 0.0;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = sin(Tmb.rotation() / 2.0);
    pose.pose.orientation.w = cos(Tmb.rotation() / 2.0);

    poses_.push_back(pose);
    msg_path.poses = poses_;

    pub_path_->publish(msg_path);
  }

  /// \brief publish the odometry
  void publish_odom_()
  {
    Odometry msg_odom;

    const auto phi_left = joint_states_curr_.position.at(index_left_);
    const auto phi_right = joint_states_curr_.position.at(index_right_);

    turtlelib::Twist2D twist_turtle = turtlebot_.compute_fk(phi_left, phi_right);

    msg_odom.header.stamp = get_clock()->now();
    msg_odom.header.frame_id = odom_id_;
    msg_odom.child_frame_id = body_id_;

    msg_odom.pose.pose.position.x = turtlebot_.config_x();
    msg_odom.pose.pose.position.y = turtlebot_.config_y();
    msg_odom.pose.pose.position.z = 0.0;

    msg_odom.pose.pose.orientation.x = 0.0;
    msg_odom.pose.pose.orientation.y = 0.0;
    msg_odom.pose.pose.orientation.z = sin(turtlebot_.config_theta() / 2.0);
    msg_odom.pose.pose.orientation.w = cos(turtlebot_.config_theta() / 2.0);

    const auto t_prev = static_cast<double>(joint_states_prev_.header.stamp.sec) +
      static_cast<double>(joint_states_prev_.header.stamp.nanosec) * 1e-9;
    const auto t_curr = static_cast<double>(joint_states_curr_.header.stamp.sec) +
      static_cast<double>(joint_states_curr_.header.stamp.nanosec) * 1e-9;
    const auto dt = t_curr - t_prev;

    msg_odom.twist.twist.linear.x = twist_turtle.x / dt;
    msg_odom.twist.twist.linear.y = twist_turtle.y / dt;
    msg_odom.twist.twist.linear.z = 0.0;

    msg_odom.twist.twist.angular.x = 0.0;
    msg_odom.twist.twist.angular.y = 0.0;
    msg_odom.twist.twist.angular.z = twist_turtle.omega / dt;

    pub_odometry_->publish(msg_odom);
  }

  /// \brief Broadcast the transform
  void broadcast_tf_()
  {
    TransformStamped tf_map_odom;

    tf_map_odom.header.stamp = get_clock()->now();
    tf_map_odom.header.frame_id = map_id_;
    tf_map_odom.child_frame_id = odom_id_;

    tf_map_odom.transform.translation.x = Tmo_.translation().x;
    tf_map_odom.transform.translation.y = Tmo_.translation().y;
    tf_map_odom.transform.translation.z = 0.0;

    tf_map_odom.transform.rotation.x = 0.0;
    tf_map_odom.transform.rotation.y = 0.0;
    tf_map_odom.transform.rotation.z = sin(Tmo_.rotation() / 2.0);
    tf_map_odom.transform.rotation.w = cos(Tmo_.rotation() / 2.0);

    TransformStamped tf_odom_body;

    tf_odom_body.header.stamp = get_clock()->now();
    tf_odom_body.header.frame_id = odom_id_;
    tf_odom_body.child_frame_id = body_id_;

    tf_odom_body.transform.translation.x = turtlebot_.config_x();
    tf_odom_body.transform.translation.y = turtlebot_.config_y();
    tf_odom_body.transform.translation.z = 0.0;

    tf_odom_body.transform.rotation.x = 0.0;
    tf_odom_body.transform.rotation.y = 0.0;
    tf_odom_body.transform.rotation.z = sin(turtlebot_.config_theta() / 2.0);
    tf_odom_body.transform.rotation.w = cos(turtlebot_.config_theta() / 2.0);

    tf_broadcater_->sendTransform(tf_map_odom);
    tf_broadcater_->sendTransform(tf_odom_body);
  }

  /// \brief Update the transform from map to odom based on new configuration
  /// \param x_map x configuration in map frame
  /// \param y_map y configuration in map frame
  /// \param theta_map theta configuration in map frame
  void update_map_odom_tf_(double x_map, double y_map, double theta_map)
  {
    const auto x_odom = turtlebot_.config_x();
    const auto y_odom = turtlebot_.config_y();
    const auto theta_odom = turtlebot_.config_theta();

    const turtlelib::Transform2D Tmb{{x_map, y_map}, theta_map};
    const turtlelib::Transform2D Tob{{x_odom, y_odom}, theta_odom};
    const turtlelib::Transform2D Tbo = Tob.inv();

    Tmo_ = Tmb * Tbo;
  }

  /// Timer
  rclcpp::TimerBase::SharedPtr timer_;

  /// Subscriber
  rclcpp::Subscription<JointState>::SharedPtr sub_joint_states_;
  rclcpp::Subscription<ObstacleMeasurements>::SharedPtr sub_obs_measure_;

  /// Publisher
  rclcpp::Publisher<Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<Path>::SharedPtr pub_path_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_map_array_;

  /// TF Broadcaster
  std::unique_ptr<TransformBroadcaster> tf_broadcater_;

  /// Service
  rclcpp::Service<InitialPose>::SharedPtr srv_initial_pose_;

  /// QoS
  rclcpp::QoS marker_qos_;
  rclcpp::QoS path_qos_;

  /// Subscribed messages
  JointState joint_states_curr_;
  JointState joint_states_prev_;
  Point odom_position_;
  Quaternion odom_orientation_;
  ObstacleMeasurements obs_measure_;

  /// parameters
  std::string body_id_;
  std::string odom_id_;
  std::string map_id_;
  std::string wheel_left_;
  std::string wheel_right_;
  double track_width_;
  double wheel_radius_;
  double input_noice_;
  double sensor_noice_;

  /// other attributes
  bool joint_states_available_;
  size_t index_left_;
  size_t index_right_;
  double left_init_;
  double right_init_;
  std::vector<PoseStamped> poses_;
  arma::mat Q_mat_;
  int num_obstacles_;
  turtlelib::DiffDrive turtlebot_;
  turtlelib::EKF turtle_slam_;
  std::default_random_engine generator_;
  std::normal_distribution<double> dist_sensor_;
  double marker_radius_;
  double marker_height_;
  double marker_r_;
  double marker_g_;
  double marker_b_;
  turtlelib::Transform2D Tmo_;
  bool landmark_updated_;

  /// Constants
  const size_t MAX_PATH_LEN = 100;

public:
  /// \brief
  Slam()
  : Node("odometry"), marker_qos_(10), path_qos_(10), joint_states_available_(false),
    index_left_(SIZE_MAX), index_right_(SIZE_MAX), num_obstacles_(20), turtle_slam_(num_obstacles_),
    marker_radius_(0.038), marker_height_(0.25), Tmo_({0.0, 0.0}, 0.0), landmark_updated_(false)
  {
    ParameterDescriptor body_id_des;
    ParameterDescriptor odom_id_des;
    ParameterDescriptor map_id_des;
    ParameterDescriptor wheel_left_des;
    ParameterDescriptor wheel_right_des;
    ParameterDescriptor wheel_radius_des;
    ParameterDescriptor track_width_des;
    ParameterDescriptor input_noice_des;
    ParameterDescriptor sensor_noice_des;

    body_id_des.description = "The name of the body frame of the robot.";
    odom_id_des.description = "The name of the odometry frame.";
    map_id_des.description = "The name of the map frame";
    wheel_left_des.description = "The name of the left wheel joint.";
    wheel_right_des.description = "The name of the right wheel joint.";
    wheel_radius_des.description = "Wheel radius of the turtlebot.";
    track_width_des.description = "Track width of the turtlebot.";
    input_noice_des.description = "Input noice of the robot";
    sensor_noice_des.description = "Sensor noice of the robot";

    declare_parameter<std::string>("body_id", "", body_id_des);
    declare_parameter<std::string>("odom_id", "odom", odom_id_des);
    declare_parameter<std::string>("map_id", "map", map_id_des);
    declare_parameter<std::string>("wheel_left", "", wheel_left_des);
    declare_parameter<std::string>("wheel_right", "", wheel_right_des);
    declare_parameter<double>("wheel_radius", 0.033, wheel_radius_des);
    declare_parameter<double>("track_width", 0.16, track_width_des);
    declare_parameter<double>("input_noice", 0.1, input_noice_des);
    declare_parameter<double>("basic_sensor_variance", 0.1, sensor_noice_des);

    body_id_ = get_parameter("body_id").as_string();
    odom_id_ = get_parameter("odom_id").as_string();
    map_id_ = get_parameter("map_id").as_string();
    wheel_left_ = get_parameter("wheel_left").as_string();
    wheel_right_ = get_parameter("wheel_right").as_string();
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    input_noice_ = get_parameter("input_noice").as_double();
    sensor_noice_ = get_parameter("basic_sensor_variance").as_double();

    dist_sensor_ = std::normal_distribution<double>(0.0, sqrt(sensor_noice_));
    turtlebot_ = turtlelib::DiffDrive(track_width_, wheel_radius_);

    const arma::mat upper_left = arma::mat(3, 3, arma::fill::eye) * input_noice_;
    const arma::mat upper_right(3, 2 * num_obstacles_, arma::fill::zeros);
    const arma::mat bottom_left(2 * num_obstacles_, 3, arma::fill::zeros);
    const arma::mat bottom_right(2 * num_obstacles_, 2 * num_obstacles_, arma::fill::zeros);

    Q_mat_ = arma::join_horiz(
      arma::join_vert(upper_left, bottom_left),
      arma::join_vert(upper_right, bottom_right));

    if (body_id_.size() == 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid body id: " << body_id_);
      exit(EXIT_FAILURE);
    }

    if (wheel_left_.size() == 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid left wheel: " << wheel_left_);
      exit(EXIT_FAILURE);
    }

    if (wheel_right_.size() == 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid right wheel: " << wheel_right_);
      exit(EXIT_FAILURE);
    }

    /// QoS
    marker_qos_.transient_local();
    path_qos_.transient_local();

    /// Transform broadcaster
    tf_broadcater_ = std::make_unique<TransformBroadcaster>(*this);

    /// Timer
    timer_ = create_wall_timer(5ms, std::bind(&Slam::timer_callback_, this));

    /// Subscriptions
    sub_joint_states_ =
      create_subscription<JointState>(
      "joint_states",
      10,
      std::bind(
        &Slam::sub_joint_states_callback_,
        this,
        std::placeholders::_1));
    sub_obs_measure_ =
      create_subscription<ObstacleMeasurements>(
      "obs_pos",
      10,
      std::bind(
        &Slam::sub_obs_measure_callback_,
        this,
        std::placeholders::_1));

    /// Publishers
    pub_odometry_ = create_publisher<Odometry>("odom", 10);
    pub_path_ = create_publisher<Path>("~/path", path_qos_);
    pub_map_array_ = create_publisher<MarkerArray>("~/map", marker_qos_);

    /// Services
    srv_initial_pose_ =
      create_service<InitialPose>(
      "initial_pose",
      std::bind(
        &Slam::srv_initial_pose_callback_,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  }
};

/// \brief The main entry of the node
/// \param argc The number of arguments
/// \param argv The value of arguments
/// \return The result code.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_slam = std::make_shared<Slam>();
  rclcpp::spin(node_slam);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
