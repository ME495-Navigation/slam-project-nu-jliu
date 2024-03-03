#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "turtlelib/detect.hpp"

using rcl_interfaces::msg::ParameterDescriptor;
using sensor_msgs::msg::LaserScan;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;


class Landmark : public rclcpp::Node
{
private:
  void sub_scan_callback_(LaserScan::SharedPtr msg)
  {
    scan_frame_id_ = msg->header.frame_id;
    const auto angle_min = msg->angle_min;
    const auto angle_inc = msg->angle_increment;
    const auto data = msg->ranges;

    std::vector<turtlelib::Point2D> data_points;

    for (size_t i = 0; i < data.size(); ++i) {
      const auto theta = angle_min + i * angle_inc;
      const auto range = data.at(i);

      const auto x = range * cos(theta);
      const auto y = range * sin(theta);

      const turtlelib::Point2D point{x, y};
      data_points.push_back(point);

      RCLCPP_INFO_STREAM(get_logger(), point);
    }

    publish_markers_();
  }

  void publish_markers_()
  {
    MarkerArray msg_markers;

    Marker m;

    m.header.stamp = get_clock()->now();
    m.header.frame_id = scan_frame_id_;
    m.id = 40;
    m.type = Marker::CYLINDER;
    m.action = Marker::ADD;
    m.pose.position.x = 1.0;
    m.pose.position.y = 1.0;
    m.pose.position.z = 0.0;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a = 0.5;

    msg_markers.markers.push_back(m);
    pub_detect_marker_->publish(msg_markers);
  }

  rclcpp::QoS marker_qos_;

  rclcpp::Subscription<LaserScan>::SharedPtr sub_scan_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_detect_marker_;

  std::string scan_frame_id_;

public:
  Landmark()
  : rclcpp::Node("landmark"), marker_qos_(10)
  {
    marker_qos_.transient_local();

    sub_scan_ =
      create_subscription<LaserScan>(
      "scan", 10,
      std::bind(&Landmark::sub_scan_callback_, this, std::placeholders::_1));

    pub_detect_marker_ = create_publisher<MarkerArray>("~/detect", marker_qos_);
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_landmark = std::make_shared<Landmark>();
  rclcpp::spin(node_landmark);

  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
