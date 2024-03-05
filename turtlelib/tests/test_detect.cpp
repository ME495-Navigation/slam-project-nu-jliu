#include <catch2/catch_all.hpp>
#include <vector>

#include "turtlelib/detect.hpp"

#define TOLERANCE 1e-10

using Catch::Matchers::WithinAbs;

namespace turtlelib
{
TEST_CASE("Test Constructor", "[CircleDetect]")
{
  CircleDetect detect_test1;
  const auto data_test1 = detect_test1.get_data_points();

  CircleDetect detect_test2(3);
  const auto data_test2 = detect_test2.get_data_points();

  REQUIRE(data_test1.size() == 0);
  REQUIRE(data_test2.size() == 3);

  REQUIRE_THAT(data_test2.at(0).x, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(data_test2.at(0).y, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(data_test2.at(1).x, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(data_test2.at(1).y, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(data_test2.at(2).x, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(data_test2.at(2).y, WithinAbs(0.0, TOLERANCE));
}

TEST_CASE("Test Update Data", "[update_data]") {
  std::vector<Point2D> test_data{{1.2, 3.4}, {-1.3, 4.6}, {-3.3, -0.9}};

  CircleDetect detect_test;
  detect_test.update_data(test_data);
  const auto data_test = detect_test.get_data_points();

  REQUIRE_THAT(data_test.at(0).x, WithinAbs(1.2, TOLERANCE));
  REQUIRE_THAT(data_test.at(0).y, WithinAbs(3.4, TOLERANCE));
  REQUIRE_THAT(data_test.at(1).x, WithinAbs(-1.3, TOLERANCE));
  REQUIRE_THAT(data_test.at(1).y, WithinAbs(4.6, TOLERANCE));
  REQUIRE_THAT(data_test.at(2).x, WithinAbs(-3.3, TOLERANCE));
  REQUIRE_THAT(data_test.at(2).y, WithinAbs(-0.9, TOLERANCE));
}

TEST_CASE("Test Compute x mean", "[compte_x_mean]")
{
  std::vector<Point2D> test_data1{{1.2, 3.4}, {-1.3, 4.6}, {-3.3, -0.9}};
  CircleDetect detect1(3);
  detect1.update_data(test_data1);
  double x_mean1 = detect1.compute_x_mean();

  std::vector<Point2D> test_data2{{3.6, 3.4}, {-2.5, 4.6}, {9.3, -0.9}};
  CircleDetect detect2(3);
  detect2.update_data(test_data2);
  double x_mean2 = detect2.compute_x_mean();

  REQUIRE_THAT(x_mean1, WithinAbs(-3.4 / 3.0, TOLERANCE));
  REQUIRE_THAT(x_mean2, WithinAbs(10.4 / 3.0, TOLERANCE));
}

TEST_CASE("Test compute y mean", "[compute_y_mean]")
{
  std::vector<Point2D> test_data1{{1.2, 3.4}, {-1.3, 4.6}, {-3.3, -0.9}};
  CircleDetect detect1(3);
  detect1.update_data(test_data1);
  double y_mean1 = detect1.compute_y_mean();

  std::vector<Point2D> test_data2{{3.6, -1.2}, {-2.5, -4.6}, {9.3, -9.9}};
  CircleDetect detect2(3);
  detect2.update_data(test_data2);
  double y_mean2 = detect2.compute_y_mean();

  REQUIRE_THAT(y_mean1, WithinAbs(7.1 / 3.0, TOLERANCE));
  REQUIRE_THAT(y_mean2, WithinAbs(-15.7 / 3.0, TOLERANCE));
}

TEST_CASE("Test shift coordinate", "[shift_coordinate]")
{
  std::vector<Point2D> test_data{{1.2, 3.4}, {-1.3, 4.6}, {-3.3, -0.9}};

  CircleDetect detect_test(3);
  detect_test.update_data(test_data);
  const auto x_mean = detect_test.compute_x_mean();
  const auto y_mean = detect_test.compute_y_mean();

  detect_test.shift_coordinate(x_mean, y_mean);
  const auto data_test = detect_test.get_data_points();

  REQUIRE_THAT(data_test.at(0).x, WithinAbs(1.2 - x_mean, TOLERANCE));
  REQUIRE_THAT(data_test.at(0).y, WithinAbs(3.4 - y_mean, TOLERANCE));
  REQUIRE_THAT(data_test.at(1).x, WithinAbs(-1.3 - x_mean, TOLERANCE));
  REQUIRE_THAT(data_test.at(1).y, WithinAbs(4.6 - y_mean, TOLERANCE));
  REQUIRE_THAT(data_test.at(2).x, WithinAbs(-3.3 - x_mean, TOLERANCE));
  REQUIRE_THAT(data_test.at(2).y, WithinAbs(-0.9 - y_mean, TOLERANCE));

  const auto x_mean_new = detect_test.compute_x_mean();
  const auto y_mean_new = detect_test.compute_y_mean();

  REQUIRE_THAT(x_mean_new, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(y_mean_new, WithinAbs(0.0, TOLERANCE));
}

TEST_CASE("Test Compute z mean", "[compute_z_mean]")
{
  std::vector<Point2D> test_data1{{1.2, 3.4}, {-1.3, 4.6}, {-3.3, -0.9}};
  CircleDetect detect1(3);
  detect1.update_data(test_data1);
  const auto z_mean1 = detect1.compute_z_mean();

  std::vector<Point2D> test_data2{{3.6, -1.2}, {-2.5, -4.6}, {9.3, -9.9}};
  CircleDetect detect2(3);
  detect2.update_data(test_data2);
  const auto z_mean2 = detect2.compute_z_mean();

  REQUIRE_THAT(z_mean1, WithinAbs(47.55 / 3.0, TOLERANCE));
  REQUIRE_THAT(z_mean2, WithinAbs(226.31 / 3.0, TOLERANCE));
}

TEST_CASE("Test Z matrix", "[construct_Z_mat]")
{
  const auto tolerance = 1e-4;

  std::vector<Point2D> test_data1{{1.2, 3.4}, {-1.3, 4.6}, {-3.3, -0.9}};
  CircleDetect detect1(3);
  detect1.update_data(test_data1);

  const auto x_mean1 = detect1.compute_x_mean();
  const auto y_mean1 = detect1.compute_y_mean();
  detect1.shift_coordinate(x_mean1, y_mean1);

  const arma::mat Z_mat1 = detect1.construct_Z_mat();

  std::vector<Point2D> test_data2{{3.6, -1.2}, {-2.5, -4.6}, {9.3, -9.9}};
  CircleDetect detect2(3);
  detect2.update_data(test_data2);

  const auto x_mean2 = detect2.compute_x_mean();
  const auto y_mean2 = detect2.compute_y_mean();
  detect2.shift_coordinate(x_mean2, y_mean2);

  const arma::mat Z_mat2 = detect2.construct_Z_mat();

  REQUIRE(Z_mat1.size() == 3 * 4);
  REQUIRE_THAT(Z_mat1(0, 0), WithinAbs(6.5122, tolerance));
  REQUIRE_THAT(Z_mat1(0, 1), WithinAbs(2.3333, tolerance));
  REQUIRE_THAT(Z_mat1(0, 2), WithinAbs(1.0333, tolerance));
  REQUIRE_THAT(Z_mat1(0, 3), WithinAbs(1.0000, tolerance));

  REQUIRE_THAT(Z_mat1(1, 0), WithinAbs(5.0156, tolerance));
  REQUIRE_THAT(Z_mat1(1, 1), WithinAbs(-0.1667, tolerance));
  REQUIRE_THAT(Z_mat1(1, 2), WithinAbs(2.2333, tolerance));
  REQUIRE_THAT(Z_mat1(1, 3), WithinAbs(1.0000, tolerance));

  REQUIRE_THAT(Z_mat1(2, 0), WithinAbs(15.3656, tolerance));
  REQUIRE_THAT(Z_mat1(2, 1), WithinAbs(-2.1667, tolerance));
  REQUIRE_THAT(Z_mat1(2, 2), WithinAbs(-3.2667, tolerance));
  REQUIRE_THAT(Z_mat1(2, 3), WithinAbs(1.0000, tolerance));

  REQUIRE(Z_mat2.size() == 3 * 4);

  REQUIRE_THAT(Z_mat2(0, 0), WithinAbs(16.2856, tolerance));
  REQUIRE_THAT(Z_mat2(0, 1), WithinAbs(0.1333, tolerance));
  REQUIRE_THAT(Z_mat2(0, 2), WithinAbs(4.0333, tolerance));
  REQUIRE_THAT(Z_mat2(0, 3), WithinAbs(1.0000, tolerance));

  REQUIRE_THAT(Z_mat2(1, 0), WithinAbs(36.0022, tolerance));
  REQUIRE_THAT(Z_mat2(1, 1), WithinAbs(-5.9667, tolerance));
  REQUIRE_THAT(Z_mat2(1, 2), WithinAbs(0.6333, tolerance));
  REQUIRE_THAT(Z_mat2(1, 3), WithinAbs(1.0000, tolerance));

  REQUIRE_THAT(Z_mat2(2, 0), WithinAbs(55.8056, tolerance));
  REQUIRE_THAT(Z_mat2(2, 1), WithinAbs(5.8333, tolerance));
  REQUIRE_THAT(Z_mat2(2, 2), WithinAbs(-4.6667, tolerance));
  REQUIRE_THAT(Z_mat2(2, 3), WithinAbs(1.0000, tolerance));
}

TEST_CASE("Test M matrix", "[compute_M_mat]")
{
  std::vector<Point2D> test_data1{{1.2, 3.4}, {-1.3, 4.6}, {-3.3, -0.9}};
  CircleDetect detect1(3);
  detect1.update_data(test_data1);

  const auto x_mean1 = detect1.compute_x_mean();
  const auto y_mean1 = detect1.compute_y_mean();
  detect1.shift_coordinate(x_mean1, y_mean1);

  const arma::mat Z_mat1 = detect1.construct_Z_mat();
  const arma::mat M_mat1 = detect1.compute_M_mat(Z_mat1);

  std::vector<Point2D> test_data2{{3.6, -1.2}, {-2.5, -4.6}, {9.3, -9.9}};
  CircleDetect detect2(3);
  detect2.update_data(test_data2);

  const auto x_mean2 = detect2.compute_x_mean();
  const auto y_mean2 = detect2.compute_y_mean();
  detect2.shift_coordinate(x_mean2, y_mean2);

  const arma::mat Z_mat2 = detect2.construct_Z_mat();
  const arma::mat M_mat2 = detect2.compute_M_mat(Z_mat2);

  REQUIRE(M_mat1.size() == 4 * 4);
  REQUIRE_THAT(M_mat1(0, 0), WithinAbs(101.2217, 1e-4));
  REQUIRE_THAT(M_mat1(0, 1), WithinAbs(-6.3109, 1e-4));
  REQUIRE_THAT(M_mat1(0, 2), WithinAbs(-10.7545, 1e-4));
  REQUIRE_THAT(M_mat1(0, 3), WithinAbs(8.9644, 1e-4));

  REQUIRE_THAT(M_mat1(1, 0), WithinAbs(-6.3109, 1e-4));
  REQUIRE_THAT(M_mat1(1, 1), WithinAbs(3.3889, 1e-4));
  REQUIRE_THAT(M_mat1(1, 2), WithinAbs(3.0389, 1e-4));
  REQUIRE_THAT(M_mat1(1, 3), WithinAbs(0.0, 1e-4));

  REQUIRE_THAT(M_mat1(2, 0), WithinAbs(-10.7545, 1e-4));
  REQUIRE_THAT(M_mat1(2, 1), WithinAbs(3.0389, 1e-4));
  REQUIRE_THAT(M_mat1(2, 2), WithinAbs(5.5756, 1e-4));
  REQUIRE_THAT(M_mat1(2, 3), WithinAbs(0.0, 1e-4));

  REQUIRE_THAT(M_mat1(3, 0), WithinAbs(8.9644, 1e-4));
  REQUIRE_THAT(M_mat1(3, 1), WithinAbs(0.0, 1e-4));
  REQUIRE_THAT(M_mat1(3, 2), WithinAbs(0.0, 1e-4));
  REQUIRE_THAT(M_mat1(3, 3), WithinAbs(1.0, 1e-4));

  REQUIRE(M_mat2.size() == 4 * 4);
  REQUIRE_THAT(M_mat2(0, 0), WithinAbs(1558.5465, 1e-4));
  REQUIRE_THAT(M_mat2(0, 1), WithinAbs(37.6302, 1e-4));
  REQUIRE_THAT(M_mat2(0, 2), WithinAbs(-57.3131, 1e-4));
  REQUIRE_THAT(M_mat2(0, 3), WithinAbs(36.0311, 1e-4));

  REQUIRE_THAT(M_mat2(1, 0), WithinAbs(37.6302, 1e-4));
  REQUIRE_THAT(M_mat2(1, 1), WithinAbs(23.2156, 1e-4));
  REQUIRE_THAT(M_mat2(1, 2), WithinAbs(-10.1544, 1e-4));
  REQUIRE_THAT(M_mat2(1, 3), WithinAbs(0.0, 1e-4));

  REQUIRE_THAT(M_mat2(2, 0), WithinAbs(-57.3131, 1e-4));
  REQUIRE_THAT(M_mat2(2, 1), WithinAbs(-10.1544, 1e-4));
  REQUIRE_THAT(M_mat2(2, 2), WithinAbs(12.8156, 1e-4));
  REQUIRE_THAT(M_mat2(2, 3), WithinAbs(0.0, 1e-4));

  REQUIRE_THAT(M_mat2(3, 0), WithinAbs(36.0311, 1e-4));
  REQUIRE_THAT(M_mat2(3, 1), WithinAbs(0.0, 1e-4));
  REQUIRE_THAT(M_mat2(3, 2), WithinAbs(0.0, 1e-4));
  REQUIRE_THAT(M_mat2(3, 3), WithinAbs(1.0, 1e-4));
}

TEST_CASE("Test Circle Detection", "[detect_circle]")
{
  std::vector<Point2D> data1{
    {1.0, 7.0},
    {2.0, 6.0},
    {5.0, 8.0},
    {7.0, 7.0},
    {9.0, 5.0},
    {3.0, 7.0}
  };

  std::vector<Point2D> data2{
    {-1.0, 0.0},
    {-0.3, -0.06},
    {0.3, 0.1},
    {1.0, 1.0}
  };

  CircleDetect detet;

  detet.update_data(data1);
  const Landmark lm1 = detet.detect_circle();

  detet.update_data(data2);
  const Landmark lm2 = detet.detect_circle();

  REQUIRE_THAT(lm1.x, WithinAbs(4.615482, 1e-4));
  REQUIRE_THAT(lm1.y, WithinAbs(2.807354, 1e-4));
  REQUIRE_THAT(lm1.r, WithinAbs(4.8275, 1e-4));

  REQUIRE_THAT(lm2.x, WithinAbs(0.4908357, 1e-4));
  REQUIRE_THAT(lm2.y, WithinAbs(-22.15212, 1e-4));
  REQUIRE_THAT(lm2.r, WithinAbs(22.17979, 1e-4));
  // std::cout << lm1.x << " " << lm1.y << " " << lm1.r << std::endl;
}
} /// namespace turtlelib
