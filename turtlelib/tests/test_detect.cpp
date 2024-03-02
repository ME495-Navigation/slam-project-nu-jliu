#include <catch2/catch_all.hpp>
#include <vector>

#include "turtlelib/detect.hpp"

#define TOLERANCE 1e-10

using Catch::Matchers::WithinAbs;

namespace turtlelib
{
TEST_CASE("Test Constructor", "[CircleDetect]")
{
  std::vector<Point2D> test_data{{1.2, 3.4}, {-1.3, 4.6}, {-3.3, -0.9}};

  CircleDetect detect_test(test_data);
  const auto data_test = detect_test.get_data_points();

  REQUIRE_THAT(data_test.at(0).x, WithinAbs(1.2, TOLERANCE));
  REQUIRE_THAT(data_test.at(0).y, WithinAbs(3.4, TOLERANCE));
  REQUIRE_THAT(data_test.at(1).x, WithinAbs(-1.3, TOLERANCE));
  REQUIRE_THAT(data_test.at(1).y, WithinAbs(4.6, TOLERANCE));
  REQUIRE_THAT(data_test.at(2).x, WithinAbs(-3.3, TOLERANCE));
  REQUIRE_THAT(data_test.at(2).y, WithinAbs(-0.9, TOLERANCE));
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
  CircleDetect detect1(test_data1);
  double x_mean1 = detect1.compute_x_mean();

  std::vector<Point2D> test_data2{{3.6, 3.4}, {-2.5, 4.6}, {9.3, -0.9}};
  CircleDetect detect2(test_data2);
  double x_mean2 = detect2.compute_x_mean();

  REQUIRE_THAT(x_mean1, WithinAbs(-3.4 / 3.0, TOLERANCE));
  REQUIRE_THAT(x_mean2, WithinAbs(10.4 / 3.0, TOLERANCE));
}

TEST_CASE("Test compute y mean", "[compute_y_mean]")
{
  std::vector<Point2D> test_data1{{1.2, 3.4}, {-1.3, 4.6}, {-3.3, -0.9}};
  CircleDetect detect1(test_data1);
  double y_mean1 = detect1.compute_y_mean();

  std::vector<Point2D> test_data2{{3.6, -1.2}, {-2.5, -4.6}, {9.3, -9.9}};
  CircleDetect detect2(test_data2);
  double y_mean2 = detect2.compute_y_mean();

  REQUIRE_THAT(y_mean1, WithinAbs(7.1 / 3.0, TOLERANCE));
  REQUIRE_THAT(y_mean2, WithinAbs(-15.7 / 3.0, TOLERANCE));
}

TEST_CASE("Test shift coordinate", "[shift_coordinate]")
{
  std::vector<Point2D> test_data{{1.2, 3.4}, {-1.3, 4.6}, {-3.3, -0.9}};

  CircleDetect detect_test(test_data);
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
} /// namespace turtlelib
