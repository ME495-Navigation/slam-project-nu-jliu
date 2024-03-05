#ifndef DETECT_HPP_INCLUDE_GUARD
#define DETECT_HPP_INCLUDE_GUARD

#include <vector>
#include <armadillo>

#include "turtlelib/se2d.hpp"

namespace turtlelib
{
struct Landmark
{
  double x;
  double y;
  double r;
};

class CircleDetect
{
private:
  std::vector<Point2D> data_points_;
  size_t num_data_;

public:
  CircleDetect();

  CircleDetect(const int num);

  void update_data(std::vector<Point2D> & data);

  double compute_x_mean();

  double compute_y_mean();

  void shift_coordinate(const double x_mean, const double y_mean);

  double compute_z_mean();

  arma::mat construct_Z_mat();

  arma::mat compute_M_mat(const arma::mat & Z_mat);

  arma::mat construct_H_mat(const double z_mean);

  Landmark detect_circle();

  std::vector<Point2D> & get_data_points();
};
} // namespace turtlelib

#endif
