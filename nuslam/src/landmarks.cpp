#include <iostream>

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
    const auto range_min = msg->range_min;
    const auto range_max = msg->range_max;

    std::vector<turtlelib::Point2D> data_points;

    for (size_t i = 0; i < data.size(); ++i) {
      const auto theta = angle_min + i * angle_inc;
      const auto range = data.at(i);

      const auto x = range * cos(theta);
      const auto y = range * sin(theta);

      if (range > range_min && range < range_max) {

        const turtlelib::Point2D point{x, y};
        data_points.push_back(point);

        RCLCPP_INFO_STREAM(get_logger(), point);
      } else if (data_points.size() > 0) {
        detection_.update_data(data_points);

        const double x_hat = detection_.compute_x_mean();
        const double y_hat = detection_.compute_y_mean();

        detection_.shift_coordinate(x_hat, y_hat);
        const double z_bar = detection_.compute_z_mean();

        const arma::mat Z_mat = detection_.construct_Z_mat();
        const arma::mat M_mat = detection_.compute_M_mat(Z_mat);
        const arma::mat H_mat = detection_.construct_H_mat(z_bar);
        const arma::mat H_inv = H_mat.i();

        arma::mat U_mat;
        arma::vec sigma_vec;
        arma::mat V_mat;

        arma::svd(U_mat, sigma_vec, V_mat, Z_mat);
        const arma::mat Sigma_mat = arma::diagmat(sigma_vec);

        RCLCPP_INFO_STREAM(get_logger(), "sigma_vec: " << std::endl << sigma_vec);
        RCLCPP_INFO_STREAM(get_logger(), "Sigma_mat: " << std::endl << Sigma_mat);

        const double sigma_min = sigma_vec.min();
        RCLCPP_INFO_STREAM(get_logger(), "sigma_min: " << sigma_min);

        arma::mat A_vec;
        if (sigma_min < 1e-12) {
          A_vec = V_mat.col(3);
        } else {
          const arma::mat Y_mat = V_mat * Sigma_mat * V_mat.t();
          const arma::mat Q_mat = Y_mat * H_inv * Y_mat;

          arma::vec eig_val;
          arma::mat eig_vec;
          arma::eig_sym(eig_val, eig_vec, Q_mat);
          RCLCPP_INFO_STREAM(get_logger(), "Eigen value: " << std::endl << eig_vec << eig_val);

          const int min_eig_ind = eig_val.index_min();
          const arma::vec A_star_vec = eig_vec.col(min_eig_ind);

          A_vec = arma::solve(Y_mat, A_star_vec);
        }

        RCLCPP_INFO_STREAM(get_logger(), "A_vec: " << std::endl << A_vec);

        const double A1 = A_vec(0);
        const double A2 = A_vec(1);
        const double A3 = A_vec(2);
        const double A4 = A_vec(3);

        const double a = -A2 / (2.0 * A1);
        const double b = -A3 / (2.0 * A1);
        const double R2 = (pow(A2, 2.0) + pow(A3, 2.0) - 4.0 * A1 * A4) / (4.0 * pow(A1, 2.0));

        const double center_x = a + x_hat;
        const double center_y = b + y_hat;
        const double R = sqrt(R2);

        const turtlelib::Point2D center{center_x, center_y};
        RCLCPP_INFO_STREAM(get_logger(), "fit center: " << center);
        landmarks_.push_back({center_x, center_y, R});

        data_points.clear();
      }
    }

    if (landmarks_.size() > 0) {
      publish_markers_();
      landmarks_.clear();
    }
  }

  void publish_markers_()
  {
    MarkerArray msg_markers;

    for (size_t i = 0; i < landmarks_.size(); ++i) {
      Marker m;

      m.header.stamp = get_clock()->now();
      m.header.frame_id = scan_frame_id_;
      m.id = 40 + i;
      m.type = Marker::CYLINDER;
      m.action = Marker::ADD;
      m.pose.position.x = landmarks_.at(i).x;
      m.pose.position.y = landmarks_.at(i).y;
      m.pose.position.z = marker_height_ / 2.0 - 0.182;
      m.scale.x = 2.0 * landmarks_.at(i).r;
      m.scale.y = 2.0 * landmarks_.at(i).r;
      m.scale.z = marker_height_;
      m.color.r = 0.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      m.color.a = 1.0;

      msg_markers.markers.push_back(m);
    }
    pub_detect_marker_->publish(msg_markers);
  }

  rclcpp::QoS marker_qos_;

  rclcpp::Subscription<LaserScan>::SharedPtr sub_scan_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_detect_marker_;

  std::string scan_frame_id_;

  turtlelib::CircleDetect detection_;
  std::vector<turtlelib::Landmark> landmarks_;
  double marker_radius_;
  double marker_height_;

public:
  Landmark()
  : rclcpp::Node("landmark"), marker_qos_(10), marker_radius_(0.038), marker_height_(0.25)
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
