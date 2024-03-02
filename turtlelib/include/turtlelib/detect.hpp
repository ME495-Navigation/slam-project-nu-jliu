#ifndef DETECT_HPP_INCLUDE_GUARD
#define DETECT_HPP_INCLUDE_GUARD

#include <vector>
#include <armadillo>

#include "turtlelib/se2d.hpp"

namespace turtlelib
{
class CircleDetect
{
private:
  std::vector<Point2D> data_points_;
  int num_data_;

public:
  CircleDetect();

  CircleDetect(std::vector<Point2D> & data);

  void update_data(std::vector<Point2D> & data);

  double compute_x_mean();

  double compute_y_mean();

  void shift_coordinate(double x_mean, double y_mean);

  double compute_z_mean();

  arma::mat construct_Z_mat();

  arma::mat compute_M_mat(arma::mat Z_mat);

  std::vector<Point2D> & get_data_points();
};
} // namespace turtlelib

#endif
