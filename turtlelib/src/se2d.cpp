///
/// \file se2d.cpp
/// \author Allen Liu (jingkunliu2025@northwestern.edu)
/// \brief Turtle lib library function for 2d transformation
/// \version 0.1
/// \date 2024-01-23
///
/// \copyright Copyright (c) 2024
///
///
#include "turtlelib/se2d.hpp"

#include <cmath>
#include <iostream>

namespace turtlelib
{
std::ostream & operator<<(std::ostream & os, const Twist2D & tw)
{
  os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
  return os;
}

std::istream & operator>>(std::istream & is, Twist2D & tw)
{
    char first = is.peek(); // cons auto
  bool has_brk = (first == '[');

  std::string str_w;
  std::string str_x;
  std::string str_y;

  is >> str_w;
  is >> str_x;
  is >> str_y;

  if (has_brk) {
    str_w = str_w.substr(1, str_w.length() - 1);
    str_y = str_y.substr(0, str_y.length() - 1);
  }

  tw.omega = stod(str_w);
  tw.x = stod(str_x);
  tw.y = stod(str_y);

  return is;
}

    Transform2D::Transform2D() // use initializer list and delegating constructors
{
  __twist.omega = 0.0;
  __twist.x = 0.0;
  __twist.y = 0.0;
}

Transform2D::Transform2D(Vector2D trans) // use initializer list and delegating constructors
{
  __twist.omega = 0.0;
  __twist.x = trans.x;
  __twist.y = trans.y;
}

Transform2D::Transform2D(double radians) // use initializer list and delegating constructors
{
  __twist.omega = radians;
  __twist.x = 0.0;
  __twist.y = 0.0;
}

Transform2D::Transform2D(Vector2D trans, double radians) // use initializer list and delegating constructors
{
  __twist.omega = radians;
  __twist.x = trans.x;
  __twist.y = trans.y;
}

Point2D Transform2D::operator()(Point2D p) const
{
    double c = cos(__twist.omega); // const auto
    double s = sin(__twist.omega); // const auto
  double x = __twist.x; // const auto
  double y = __twist.y; // const atuo

  double result_x = c * p.x - s * p.y + x;
  double result_y = s * p.x + c * p.y + y;

  return {result_x, result_y}; // no need for the result_x/result_y temporaries
}

Vector2D Transform2D::operator()(Vector2D v) const
{// const auto... (will stop writing, you get the idea)
  double c = cos(__twist.omega);
  double s = sin(__twist.omega);

  double result_x = c * v.x - s * v.y;
  double result_y = s * v.x + c * v.y;

  return {result_x, result_y};
}

Twist2D Transform2D::operator()(Twist2D v) const
{
  double c = cos(__twist.omega);
  double s = sin(__twist.omega);
  double x = __twist.x;
  double y = __twist.y;

  double tw_omega = v.omega;
  double tw_x = y * v.omega + c * v.x - s * v.y;
  double tw_y = -x * v.omega + s * v.x + c * v.y;

  return {tw_omega, tw_x, tw_y};
}

Transform2D Transform2D::inv() const
{
  double c = cos(__twist.omega);
  double s = sin(__twist.omega);
  double x = __twist.x;
  double y = __twist.y;

  double inv_cos = c;
  double inv_sin = -s;
  double inv_x = -(inv_cos * x - inv_sin * y);
  double inv_y = -(inv_sin * x + inv_cos * y);

  double radian = atan2(inv_sin, inv_cos);
  Vector2D v = {inv_x, inv_y};

  return Transform2D(v, radian); // can return {v, radian}, compiler already knows its a Transform2D
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
  double c1 = cos(__twist.omega);
  double s1 = sin(__twist.omega);
  double x1 = __twist.x;
  double y1 = __twist.y;

  double c2 = cos(rhs.rotation());
  double s2 = sin(rhs.rotation());
  double x2 = rhs.translation().x;
  double y2 = rhs.translation().y;

  double cos_omg = c1 * c2 - s1 * s2;
  double sin_omg = s1 * c2 + c1 * s2;
  double omega = atan2(sin_omg, cos_omg);

  double x = c1 * x2 - s1 * y2 + x1;
  double y = s1 * x2 + c1 * y2 + y1;

  __twist.omega = omega;
  __twist.x = x;
  __twist.y = y;

  return *this;
}

Vector2D Transform2D::translation() const
{
  return {__twist.x, __twist.y};
}

double Transform2D::rotation() const
{
  return __twist.omega;
}

std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
{
  os << "deg: " << rad2deg(tf.rotation());
  os << " x: " << tf.translation().x;
  os << " y: " << tf.translation().y;

  return os;
}

std::istream & operator>>(std::istream & is, Transform2D & tf)
{
  std::string deg_str;
  std::string x_str;
  std::string y_str;
  std::string temp;

  is >> deg_str;
  is >> x_str;
  is >> y_str;

  double deg = stod(deg_str);
  double x = stod(x_str);
  double y = stod(y_str);

  struct Vector2D trans = {x, y};
  tf = Transform2D(trans, deg2rad(deg));

  return is;
}

Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
{
  double c1 = cos(lhs.rotation());
  double s1 = sin(lhs.rotation());
  double x1 = lhs.translation().x;
  double y1 = lhs.translation().y;

  double c2 = cos(rhs.rotation());
  double s2 = sin(rhs.rotation());
  double x2 = rhs.translation().x;
  double y2 = rhs.translation().y;

  double cos_omg = c1 * c2 - s1 * s2;
  double sin_omg = s1 * c2 + c1 * s2;
  double omega = atan2(sin_omg, cos_omg);

  double x = c1 * x2 - s1 * y2 + x1;
  double y = s1 * x2 + c1 * y2 + y1;

  return Transform2D(Vector2D{x, y}, omega);
}

}  // namespace turtlelib
