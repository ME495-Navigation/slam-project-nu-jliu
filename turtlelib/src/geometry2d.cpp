///
/// \file geometry2d.cpp
/// \author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// \brief Turtlelib libruary function used for 2d geometry calculation
/// \version 0.1
/// \date 2024-01-23
///
/// \copyright Copyright (c) 2024
///
///
#include "turtlelib/geometry2d.hpp"

#include <iomanip>
#include <iostream>
#include <string>

namespace turtlelib
{
double normalize_angle(double rad)
{
  if ((rad > -turtlelib::PI) && (rad <= turtlelib::PI)) {
    return rad;
  } else if (rad > turtlelib::PI) {
    return turtlelib::normalize_angle(rad - 2 * turtlelib::PI);
  } else {
    return turtlelib::normalize_angle(rad + 2 * turtlelib::PI);
  }
}

std::ostream & operator<<(std::ostream & os, const Point2D & p)
{
  os << std::setprecision(15) << "[" << p.x << " " << p.y << "]";
  return os;
}

std::istream & operator>>(std::istream & is, Point2D & p)
{
  std::string x_str;
  std::string y_str;

  char first = is.peek();
  bool has_brk = (first == '[');

  is >> x_str;
  is >> y_str;

  if (has_brk) {
    x_str = x_str.substr(1, x_str.length() - 1);
    y_str = y_str.substr(0, y_str.length() - 1);
  }

  p.x = stod(x_str);
  p.y = stod(y_str);

  return is;
}

Vector2D operator-(const Point2D & head, const Point2D & tail)
{
  const auto result_x = head.x - tail.x;
  const auto result_y = head.y - tail.y;

  return {result_x, result_y};
}

Point2D operator+(const Point2D & tail, const Vector2D & disp)
{
  const auto result_x = tail.x + disp.x;
  const auto result_y = tail.y + disp.y;

  return {result_x, result_y};
}

std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
  os << "[" << v.x << " " << v.y << "]";

  return os;
}

std::istream & operator>>(std::istream & is, Vector2D & v)
{
  std::string x_str;
  std::string y_str;

  const auto first = is.peek();
  const auto has_brk = (first == '[');

  is >> x_str;
  is >> y_str;

  if (has_brk) {
    x_str = x_str.substr(1, x_str.length() - 1);
    y_str = y_str.substr(0, y_str.length() - 1);
  }

  v.x = stod(x_str);
  v.y = stod(y_str);

  return is;
}

Vector2D operator+(const Vector2D & lhs, const Vector2D & rhs)
{
  return {lhs.x + rhs.x, lhs.y + rhs.y};
}


Vector2D operator-(const Vector2D & lhs, const Vector2D & rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y};
}

Vector2D operator*(const Vector2D & lhs, const double & rhs)
{
  return {lhs.x * rhs, lhs.y * rhs};
}

Vector2D & Vector2D::operator+=(const Vector2D & rhs)
{
  this->x += rhs.x;
  this->y += rhs.y;
  return *this;
}

Vector2D & Vector2D::operator-=(const Vector2D & rhs)
{
  this->x -= rhs.x;
  this->y -= rhs.y;
  return *this;
}

Vector2D & Vector2D::operator*=(const double & rhs)
{
  this->x *= rhs;
  this->y *= rhs;
  return *this;
}
}  // namespace turtlelib
