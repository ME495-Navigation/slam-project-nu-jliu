///
/// \file svg.hpp
/// \author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// \brief The svg class used for display points and lines in svg file
/// \version 0.1
/// \date 2024-01-22
///
/// \copyright Copyright (c) 2024
///
///
#ifndef TURTLELIB_SVG_HPP_INCLUDE_GUARD
#define TURTLELIB_SVG_HPP_INCLUDE_GUARD

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#include <iostream>
#include <fstream>

namespace turtlelib
{
/// @brief The class for displaying svg on a file
class Svg
{

private:
  std::ofstream __ofs;
  Transform2D __tf_origin;
  double __origin_x;
  double __origin_y;
  double __scale;

  /// \brief Initialize the environment for the frame
  void init_svg()
  {
    __scale = 96.0;
    __origin_x = 408.0;
    __origin_y = 528.0;

    __tf_origin = Transform2D(Vector2D{__origin_x, __origin_y}, 0.0);

    __ofs <<
      "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">"
          << std::endl;
    __ofs << "<defs>" << std::endl;
    __ofs << "<marker" << std::endl;
    __ofs << "\tstyle=\"overflow:visible\"" << std::endl;
    __ofs << "\tid=\"Arrow1Sstart\"" << std::endl;
    __ofs << "\trefX=\"0.0\"" << std::endl;
    __ofs << "\trefY=\"0.0\"" << std::endl;
    __ofs << "\torient=\"auto\">" << std::endl;
    __ofs << "<path" << std::endl;
    __ofs << "\ttransform=\"scale(0.2) translate(6,0)\"" << std::endl;
    __ofs <<
      "\tstyle=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\""
          << std::endl;
    __ofs << "\td=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"" << std::endl;
    __ofs << "/>" << std::endl;
    __ofs << "</marker>" << std::endl;
    __ofs << "</defs>" << std::endl;

    __ofs << "<circle cx=\"" << __origin_x;
    __ofs << "\" cy=\"" << __origin_y;
    __ofs << "\" r=\"5\" stroke=\"black\" fill=\"black\" stroke-width=\"1\" />";
    __ofs << std::endl;
  }

  /// \brief Calculate the transform from turtlelib frame to svg frame
  /// \param p The point to transform
  Point2D __point_tf(Point2D p)
  {
    return Point2D{__origin_x + p.x * __scale, __origin_y - p.y * __scale};
  }

public:
  /// \brief Opens a default svg file.
  Svg();

  /// \brief Opens the svg file with specified filename
  /// \param filename the name of the svg file.
  explicit Svg(std::string filename);

  /// \brief Draw a line for the specified vector
  /// \param frame
  /// \param tail the tail of the vector
  /// \param v the vector to be drawn.
  /// \param color the color for the line
  void draw_line(Transform2D frame, Point2D tail, Vector2D v, std::string color);

  /// \brief Draw a point in svg file
  /// \param frame the frame to draw on
  /// \param p the point to draw
  /// \param radius the radius for the point
  /// \param color the color for the point
  void draw_point(Transform2D frame, Point2D p, int radius, std::string color);

  /// \brief Draw a frame on the svg file
  /// \param tf The frame to draw
  /// \param name The name of the frame
  void draw_frame(Transform2D tf, std::string name);

  /// \brief Finish drawing, close the svg file
  void finish();
};
}

#endif
