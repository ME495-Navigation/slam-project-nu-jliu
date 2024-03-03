///
/// \file odometry.cpp
/// \author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// \brief Track the odometry of a robot.
///
/// PARAMETERS:
///   \param body_id      [string]  The id of the body frame
///   \param odom_id      [string]  The id of the odom frame
///   \param wheel_left   [string]  The left wheel joint name
///   \param wheel_right  [string]  The right wheel joint name
///   \param wheel_radius [double]  The radius of ythe wheel
///   \param track_width  [double]  The distance between two wheels
///
/// SUBSCRIPTIONS:
///   joint_states  [sensor_msgs/msg/JointState]      The joint state of the robot.
///
/// PUBLISHERS:
///   odom          [nav_msgs/msg/Odomoetry]          The odometry of the node.
///
/// SERVICES:
///   initial_pose [nuturtle_control/srv/InitialPose] Reset the initial pose.
///
/// \version 0.1
/// \date 2024-02-03
///
/// \copyright Copyright (c) 2024
///
///
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "turtlelib/diff_drive.hpp"
// #include "nuturtle_control/srv/initial_pose.hpp"
#include "nuturtle_interfaces/srv/initial_pose.hpp"

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

// using nuturtle_control::srv::InitialPose;
using nuturtle_interfaces::srv::InitialPose;

/// @brief
class Odom : public Node
{
private:
  /// @brief The timer callback of the odometry node
  void timer_callback_()
  {
    // RCLCPP_INFO_STREAM(
    //   get_logger(),
    //   "x: " << turtlebot_.config_x() << ", y: " << turtlebot_.config_x() << " theta: " <<
    //     turtlebot_.config_theta());

    if (joint_states_available_) {
      broadcast_tf_();
      publish_odom_();
      publish_path_();
    }
  }

  /// @brief Update the turtlebot configuration based on new joint states,
  ///        then publish new odom
  /// @param msg The subscibed joint_states message.
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
        exit(EXIT_FAILURE);
      }

      turtlebot_.update_wheel(left_init_, right_init_);
    }

    joint_states_prev_ = joint_states_curr_;
    // joint_states_curr_ = *msg;
    joint_states_curr_.header.stamp = msg->header.stamp;
    joint_states_curr_.name = msg->name;
    joint_states_curr_.position = msg->position;

    // publish_odom_();
  }

  /// @brief The initial pose service
  /// @param request The initial pose service request
  /// @param respose The initial pose service response
  void srv_initial_pose_callback_(
    std::shared_ptr<InitialPose::Request> request,
    std::shared_ptr<InitialPose::Response> respose)
  {
    double x = request->x;
    double y = request->y;
    double theta = request->theta;

    turtlebot_.update_config(x, y, theta);

    respose->success = true;
  }

  void publish_path_()
  {
    Path msg_path;

    msg_path.header.stamp = get_clock()->now();
    msg_path.header.frame_id = "nusim/world";

    PoseStamped pose;

    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = "nusim/world";

    pose.pose.position.x = turtlebot_.config_x();
    pose.pose.position.y = turtlebot_.config_y();
    pose.pose.position.z = 0.0;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = sin(turtlebot_.config_theta() / 2.0);
    pose.pose.orientation.w = cos(turtlebot_.config_theta() / 2.0);

    poses_.push_back(pose);

    if (poses_.size() >= max_path_len_) {
      poses_.erase(poses_.begin());
    }

    msg_path.poses = poses_;
    pub_path_->publish(msg_path);
  }

  /// @brief publish the odometry
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

  /// @brief Broadcast the transform
  void broadcast_tf_()
  {
    TransformStamped tf_msg;

    tf_msg.header.stamp = get_clock()->now();
    tf_msg.header.frame_id = odom_id_;
    tf_msg.child_frame_id = body_id_;

    tf_msg.transform.translation.x = turtlebot_.config_x();
    tf_msg.transform.translation.y = turtlebot_.config_y();
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = sin(turtlebot_.config_theta() / 2.0);
    tf_msg.transform.rotation.w = cos(turtlebot_.config_theta() / 2.0);

    tf_broadcater_->sendTransform(tf_msg);
  }

  /// Timer
  rclcpp::TimerBase::SharedPtr timer_;

  /// Subscriber
  rclcpp::Subscription<JointState>::SharedPtr sub_joint_states_;

  /// Publisher
  rclcpp::Publisher<Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<Path>::SharedPtr pub_path_;

  /// TF Broadcaster
  std::unique_ptr<TransformBroadcaster> tf_broadcater_;

  /// Service
  rclcpp::Service<InitialPose>::SharedPtr srv_initial_pose_;

  /// QoS Policy
  rclcpp::QoS path_qos_;

  /// Subscribed messages
  JointState joint_states_curr_;
  JointState joint_states_prev_;
  Point odom_position_;
  Quaternion odom_orientation_;

  /// parameters
  std::string body_id_;
  std::string odom_id_;
  std::string wheel_left_;
  std::string wheel_right_;
  double track_width_;
  double wheel_radius_;

  /// other attributes
  size_t max_path_len_;
  turtlelib::DiffDrive turtlebot_;
  bool joint_states_available_;
  size_t index_left_;
  size_t index_right_;
  double left_init_;
  double right_init_;
  std::vector<PoseStamped> poses_;

public:
  /// @brief
  Odom()
  : Node("odometry"), path_qos_(10), max_path_len_(1000), joint_states_available_(false),
    index_left_(SIZE_MAX), index_right_(SIZE_MAX)
  {
    ParameterDescriptor body_id_des;
    ParameterDescriptor odom_id_des;
    ParameterDescriptor wheel_left_des;
    ParameterDescriptor wheel_right_des;
    ParameterDescriptor wheel_radius_des;
    ParameterDescriptor track_width_des;

    body_id_des.description = "The name of the body frame of the robot.";
    odom_id_des.description = "The name of the odometry frame.";
    wheel_left_des.description = "The name of the left wheel joint.";
    wheel_right_des.description = "The name of the right wheel joint.";
    wheel_radius_des.description = "Wheel radius of the turtlebot.";
    track_width_des.description = "Track width of the turtlebot.";

    declare_parameter<std::string>("body_id", "", body_id_des);
    declare_parameter<std::string>("odom_id", "odom", odom_id_des);
    declare_parameter<std::string>("wheel_left", "", wheel_left_des);
    declare_parameter<std::string>("wheel_right", "", wheel_right_des);
    declare_parameter<double>("wheel_radius", 0.033, wheel_radius_des);
    declare_parameter<double>("track_width", 0.16, track_width_des);

    body_id_ = get_parameter("body_id").as_string();
    odom_id_ = get_parameter("odom_id").as_string();
    wheel_left_ = get_parameter("wheel_left").as_string();
    wheel_right_ = get_parameter("wheel_right").as_string();
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();

    turtlebot_ = turtlelib::DiffDrive(track_width_, wheel_radius_);

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

    path_qos_.transient_local();

    tf_broadcater_ = std::make_unique<TransformBroadcaster>(*this);

    timer_ = create_wall_timer(4ms, std::bind(&Odom::timer_callback_, this));

    sub_joint_states_ =
      create_subscription<JointState>(
      "joint_states", 10,
      std::bind(&Odom::sub_joint_states_callback_, this, std::placeholders::_1));

    pub_odometry_ = create_publisher<Odometry>("odom", 10);
    pub_path_ = create_publisher<Path>("~/path", path_qos_);

    srv_initial_pose_ =
      create_service<InitialPose>(
      "initial_pose",
      std::bind(
        &Odom::srv_initial_pose_callback_, this, std::placeholders::_1,
        std::placeholders::_2));
  }
};

/// @brief The main entry of the node
/// @param argc The number of arguments
/// @param argv The value of arguments
/// @return The result code.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_odom = std::make_shared<Odom>();
  rclcpp::spin(node_odom);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
