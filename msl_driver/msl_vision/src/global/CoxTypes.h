
/***************************************************************************
 *  cox_types.h - Type definitions used by the Cox algorithm
 *
 *  Created:  Sat Mar 28 16:20:34 2009
 *  Copyright 2009 Christof Rath  <christof.rath@gmail.com>
 *
 *  $Id: cox_types.h 4276 2009-03-30 14:13:27Z hoppe $
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


#ifndef __COX_COX_TYPES_H__
#define __COX_COX_TYPES_H__

#include <vector>
#include <list>
#include <cmath>
#include <string>


namespace Cox
{
#ifndef M_TWO_PI
#define M_TWO_PI 6.28318530717959
#endif

class Line2D;

/** Defines a std::vector of floats */
typedef std::vector<float> float_vector_t;

/** Defines a line point. */
typedef struct line_point_struct
{
  /** Constructor.
   * @param x x-coordinate of the line point
   * @param y y-coordinate of the line point
   */
  line_point_struct(float x = 0.f, float y = 0.f)
  {
    this->x = x;
    this->y = y;
  }

  float x; /**< The x-coordinate */
  float y; /**< The y-coordinate */
} line_point_t;

/** Defines a std::list of line points */
typedef std::list<line_point_t> line_points_t;


/** Defines a field pose (position plus orientation) */
typedef struct field_pose_struct
{
  /** Consructor.
   * @param x x-coordinate of the pose
   * @param y y-coordinate of the pose
   * @param phi orientation of the pose
   */
  field_pose_struct(float x = 0.f, float y = 0.f, float phi = 0.f)
  {
    this->phi = phi;
    this->x = x;
    this->y = y;
  }

  float x;   /**< x-coordinate of the pose */
  float y;   /**< y-coordinate of the pose */
  float phi; /**< orientation of the pose */
} field_pose_t;

/** Defines an entry of the field lookup table */
typedef struct
{
  float distance; /**< The distance to the next line */
  float dx;       /**< The gradient in x-direction */
  float dy;       /**< The gradient in y-direction */
} field_lut_entry_t;

/** Defines an arc or a circle */
typedef struct circle_struct
{
  /** Constructor.
   * @param radius The radius of the arc or circle
   * @param center_x The x-coordinate of the center of the arc or circle
   * @param center_y The x-coordinate of the center of the arc or circle
   * @param start_phi The start angle of the arc
   * @param end_phi The end angle of the arc
   */
  circle_struct(float radius, float center_x, float center_y, float start_phi = 0, float end_phi = M_TWO_PI)
  {

    if (start_phi < 0) start_phi += M_PI;
    if (end_phi < 0) end_phi += M_PI;

    this->radius    = radius;
    this->center_x  = center_x;
    this->center_y  = center_y;
    this->start_phi = start_phi < end_phi ? start_phi : end_phi;
    this->end_phi   = start_phi < end_phi ? end_phi : start_phi;
  }

  /** Returns the distance of a given point to this circle
   * @param point_x The x-coordinate of the point of interest
   * @param point_y The y-coordinate of the point of interest
   * @return The distance to the circle or arc
   */
  float get_distance(float point_x, float point_y) const
  {
    float distance = 1e50;
    float dx = point_x - center_x;
    float dy = point_y - center_y;

    float angle = atan2f(dy,dx);
    if (angle < 0) angle += 2* M_PI;

    if ((angle >= start_phi) && (angle <= end_phi))
    {
      distance =fabsf(sqrt(dy * dy + dx * dx) - radius);
    }
    else
    {
      float start_x = point_x - center_x + radius * cosf(start_phi);
      float start_y = point_y - center_y + radius * sinf(start_phi);
      float end_x = point_x - center_x + radius * cosf(end_phi);
      float end_y = point_y - center_y + radius * sinf(end_phi);

      float dist_start = sqrt((start_x * start_x) + (start_y * start_y));
      float dist_end = sqrt((end_x * end_x) + (end_y * end_y));

      distance = std::min(dist_start, dist_end);
    }

    return distance;
  }

  float radius;    /**< The radius of the arc or circle */
  float center_x;  /**< The x-coordinate of the center */
  float center_y;  /**< The y-coordinate of the center */
  float start_phi; /**< The start angle of the arc */
  float end_phi;   /**< The end angle of the arc */
} circle_t;

/** Defines a std::list of lines */
typedef std::list<Line2D> field_lines_t;
/** Defines a std::list of circles or arcs */
typedef std::list<circle_t> field_circles_t;

}

#endif // __COX_COX_TYPES_H__
