#include <armadillo>
#include "turtlelib/detect.hpp"

namespace turtlelib
{
CircleDetect::CircleDetect() {}

CircleDetect::CircleDetect(std::vector<Point2D> & data)
{
  update_data(data);
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

  for (size_t i = 0; i < data_points_.size(); ++i) {
    x_total += data_points_.at(i).x;
  }

  return x_total / static_cast<double>(num_data_);
}

double CircleDetect::compute_y_mean()
{
  double y_total = 0.0;

  for (size_t i = 0; i < data_points_.size(); ++i) {
    y_total += data_points_.at(i).y;
  }

  return y_total / static_cast<double>(num_data_);
}

void CircleDetect::shift_coordinate(double x_mean, double y_mean)
{
  for (size_t i = 0; i < data_points_.size(); ++i) {
    const auto x = data_points_.at(i).x;
    const auto y = data_points_.at(i).y;

    data_points_.at(i).x = x - x_mean;
    data_points_.at(i).y = y - y_mean;
  }
}

std::vector<Point2D> & CircleDetect::get_data_points()
{
  return data_points_;
}
} // namespace tuetlrlib
