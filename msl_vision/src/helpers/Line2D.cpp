
/***************************************************************************
 *  line_2D.cpp - A simple line representation
 *
 *  Created:  Sat Mar 28 17:20:34 2009
 *  Copyright 2008 Michael Reip   <mike1802@sbox.tugraz.at>
 *            2009 Christof Rath  <christof.rath@gmail.com>
 *
 *  $Id: line_2D.cpp 4276 2009-03-30 14:13:27Z hoppe $
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */


#include "Line2D.h"

#include <cmath>

namespace Cox
{
/** @class Line2D line_2D.h
 * A simple line representation.
 *
 * @fn const line_point_t& Line2D::get_start() const
 * Returns a const reference to the start of this line segment
 * @return a const reference to the start of this line segment
 *
 * @fn const line_point_t& Line2D::get_end() const
 * Returns a const reference to the end of this line segment
 * @return a const reference to the end of this line segment
 *
 * @author Michael Reip
 * @author Christof Rath
 */

/** Constructor.
 *
 * @param start_x x-coordinate of the start of the line
 * @param start_y y-coordinate of the start of the line
 * @param end_x x-coordinate of the end of the line
 * @param end_y y-coordinate of the end of the line
 */
Line2D::Line2D(float start_x, float start_y, float end_x, float end_y)
{
  construct(start_x, start_y, end_x, end_y);
}


/** Constructor.
 *
 * @param start start point of the line
 * @param end end point of the line
 */
Line2D::Line2D(line_point_t start, line_point_t end)
{
  construct(start.x, start.y, end.x, end.y);
}


/** Dummy destructor. */
Line2D::~Line2D()
{
}

/** Calculates the helper variables
 *
 * @param start_x x-coordinate of the start of the line
 * @param start_y y-coordinate of the start of the line
 * @param end_x x-coordinate of the end of the line
 * @param end_y y-coordinate of the end of the line
 */
void Line2D::construct(float start_x, float start_y, float end_x, float end_y)
{
  __start.x = start_x;
  __start.y = start_y;
  __end.x = end_x;
  __end.y = end_y;
  __length = p2p_distance(start_x, start_y, end_x, end_y);
  __nx = (start_y - end_y) / __length;
  __ny = (end_x - start_x) / __length;

  __line_normal_constant = __nx * start_x + __ny * start_y;
  __normal_nx = __ny;
  __normal_ny = -__nx;
  __normal_line_normal_constant_start = __normal_nx * start_x + __normal_ny * start_y;
  __normal_line_normal_constant_end   = __normal_nx * end_x + __normal_ny * end_y;
}


/** Calculates the distance of a point to an infinite line by extending this line segment
 *
 * @param point_x x-coordinate of the point of interest
 * @param point_y y-coordinate of the point of interest
 * @return the normal distance
 */
float Line2D::p2inf_line_distance(float point_x, float point_y) const
{
  float scalar = __nx * point_x + __ny * point_y;
  return fabsf(scalar - __line_normal_constant);
}


/** Calculates the distance of a point to this line segment.
 * If the normal distance cannot be calculated, the distance to the start or end
 * point is returned (whichever is shorter)
 *
 * @param point_x x-coordinate of the point of interest
 * @param point_y y-coordinate of the point of interest
 * @return either the normal distance or the distance to the start or end point
 */
float Line2D::p2line_distance(float point_x, float point_y) const
{
  float d = p2inf_line_distance(point_x, point_y);
  float value_start = point_x * __normal_nx + point_y * __normal_ny
      - __normal_line_normal_constant_start;
  float value_end = point_x * __normal_nx + point_y * __normal_ny
      - __normal_line_normal_constant_end;
  if (((value_start > 0) && (value_end > 0)) || ((value_start < 0)
      && (value_end < 0))) {
    float d_1 = p2p_distance(point_x, point_y, __start.x, __start.y);
    float d_2 = p2p_distance(point_x, point_y, __end.x, __end.y);
    if (d_1 < d_2) {
      return d_1;
    }
    else {
      return d_2;
    }
  }
  else {
    return d;
  }
}

/** Distance between two points.
 *
 * @param start_x x-coordinate of the start of the line
 * @param start_y y-coordinate of the start of the line
 * @param end_x x-coordinate of the end of the line
 * @param end_y y-coordinate of the end of the line
 * @return the Euclidean distance
 */
float Line2D::p2p_distance(float start_x, float start_y, float end_x, float end_y) const
{
  float diff_x = end_x - start_x;
  float diff_y = end_y - start_y;
  return sqrt(diff_x * diff_x + diff_y * diff_y);
}

} //END: namespace Cox

