#include <armadillo>
#include <cmath>
#include "turtlelib/detect.hpp"

namespace turtlelib
{
CircleDetect::CircleDetect()
{
  CircleDetect(0);
}

CircleDetect::CircleDetect(const int num)
{
  num_data_ = num;

  data_points_.clear();
  for (int i = 0; i < num; ++i) {
    data_points_.push_back({0, 0});
  }
}

void CircleDetect::update_data(std::vector<Point2D> & data)
{
  data_points_.clear();
  num_data_ = 0;

  for (size_t i = 0; i < data.size(); ++i) {
    const auto point = data.at(i);
    data_points_.push_back(point);
    ++num_data_;
  }
}

double CircleDetect::compute_x_mean()
{
  double x_total = 0.0;

  for (size_t i = 0; i < num_data_; ++i) {
    x_total += data_points_.at(i).x;
  }

  return x_total / static_cast<double>(num_data_);
}

double CircleDetect::compute_y_mean()
{
  double y_total = 0.0;

  for (size_t i = 0; i < num_data_; ++i) {
    y_total += data_points_.at(i).y;
  }

  return y_total / static_cast<double>(num_data_);
}

void CircleDetect::shift_coordinate(const double x_mean, const double y_mean)
{
  for (size_t i = 0; i < num_data_; ++i) {
    const auto x = data_points_.at(i).x;
    const auto y = data_points_.at(i).y;

    data_points_.at(i).x = x - x_mean;
    data_points_.at(i).y = y - y_mean;
  }
}

double CircleDetect::compute_z_mean()
{
  double z_total = 0.0;

  for (size_t i = 0; i < num_data_; ++i) {
    const auto x = data_points_.at(i).x;
    const auto y = data_points_.at(i).y;

    z_total += pow(x, 2.0) + pow(y, 2.0);
  }

  return z_total / static_cast<double>(num_data_);
}

arma::mat CircleDetect::construct_Z_mat()
{
  arma::mat Z_mat(num_data_, 4, arma::fill::zeros);

  for (size_t i = 0; i < num_data_; ++i) {
    const auto x = data_points_.at(i).x;
    const auto y = data_points_.at(i).y;
    const auto z = pow(x, 2.0) + pow(y, 2.0);

    Z_mat(i, 0) = z;
    Z_mat(i, 1) = x;
    Z_mat(i, 2) = y;
    Z_mat(i, 3) = 1.0;
  }

  return Z_mat;
}

arma::mat CircleDetect::compute_M_mat(const arma::mat & Z_mat)
{
  return 1.0 / static_cast<double>(num_data_) * Z_mat.t() * Z_mat;
}

arma::mat CircleDetect::construct_H_marix(const double z_mean)
{
  arma::mat H_mat(4, 4, arma::fill::zeros);

  H_mat(0, 0) = 8 * z_mean;
  H_mat(0, 3) = 2.0;
  H_mat(1, 1) = 1.0;
  H_mat(2, 2) = 1.0;
  H_mat(3, 0) = 2.0;

  return H_mat;
}

std::vector<Point2D> & CircleDetect::get_data_points()
{
  return data_points_;
}
} // namespace tuetlrlib
