///
/// @file svg.cpp
/// @author your name (you@domain.com)
/// @brief
/// @version 0.1
/// @date 2024-01-23
///
/// @copyright Copyright (c) 2024
///
///
#include <cmath>
#include <fstream>
#include <iostream>

#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"

namespace turtlelib
{
    // This should be implemented in se2d.hpp no there...
Vector2D normalize(Vector2D v)
{
  double mag = sqrt(pow(v.x, 2.0) + pow(v.y, 2.0));
  return Vector2D{v.x / mag, v.y / mag};
}

Svg::Svg()
{
  __ofs.open("default.svg");
  init_svg();
}

Svg::Svg(const std::string filename)
// this should likely throw an exception if the filename is not opened properly.
/// Constructors should always leave the object in a valid state
{
  __ofs.open(filename);
  init_svg();
}

void Svg::draw_line(Transform2D frame, Point2D tail, Vector2D v, std::string color)
{
  tail = frame(tail);
  v = frame(v);
  Point2D head = tail + v;

  Point2D head_tf = __point_tf(head);
  Point2D tail_tf = __point_tf(tail);

  /// THe constructor should likely prevent the __ofs from not being open at this point
  if (__ofs.is_open()) {
    __ofs << "<line x1=\"" << head_tf.x << "\" x2=\"" << tail_tf.x << "\" ";
    __ofs << "y1=\"" << head_tf.y << "\" y2=\"" << tail_tf.y << "\" ";
    __ofs << "stroke=\"" << color;
    __ofs << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />";
    __ofs << std::endl;
  }
}

void Svg::draw_point(Transform2D frame, Point2D p, int radius, std::string color)
{
  p = frame(p);
  Point2D p_tf = __point_tf(p);

  if (__ofs.is_open()) {
    __ofs << "<circle cx=\"";
    __ofs << p_tf.x << "\" ";
    __ofs << "cy=\"" << p_tf.y << "\" ";
    __ofs << "r=\"" << radius << "\" ";
    __ofs << "stroke=\"" << color << "\" fill=\"" << color
          << "\" stroke-width=\"1\" />";
    __ofs << std::endl;
  }
}

void Svg::draw_frame(Transform2D tf, std::string name)
{
//   Vector2D v_x = {-sqrt(2.0) / 2.0, -sqrt(2.0) / 2.0};
  Vector2D v_x = {1.0, 0.0};
  Vector2D v_y = {0.0, 1.0};

  Point2D origin{tf.translation().x, tf.translation().y};
  Point2D origin_tf = __point_tf(origin);

  Svg::draw_line(tf, Point2D{0.0, 0.0}, v_x, "red");
  Svg::draw_line(tf, Point2D{0.0, 0.0}, v_y, "green");
//   Svg::draw_line(tf, Point2D{0.0, 0.0}, v_z, "blue");

  if (__ofs.is_open()) {
    __ofs << "<text x=\"" << origin_tf.x << "\" ";
    __ofs << "y=\"" << origin_tf.y << "\"";
    __ofs << ">" << name << "</text>";
    __ofs << std::endl;
  }
}

void Svg::finish()
{
  __ofs << "</svg>";
  __ofs.close();
}

}  // namespace turtlelib
