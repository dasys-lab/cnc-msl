
/***************************************************************************
 *  line_2D.h - A simple line representation
 *
 *  Created:  Sat Mar 28 17:20:34 2009
 *  Copyright 2008 Michael Reip   <mike1802@sbox.tugraz.at>
 *            2009 Christof Rath  <christof.rath@gmail.com>
 *
 *  $Id: line_2D.h 4276 2009-03-30 14:13:27Z hoppe $
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


#ifndef __COX_LINE_2D_H__
#define __COX_LINE_2D_H__

#include "../global/CoxTypes.h"

namespace Cox
{
class Line2D
{
public:
  Line2D(float start_x, float start_y, float end_x, float end_y);
  Line2D(line_point_t start, line_point_t end);
  virtual ~Line2D();

  float p2line_distance(float point_x, float point_y) const;
  const line_point_t& get_start() const { return __start; }
  const line_point_t& get_end() const   { return __end; }

private:
  inline void construct(float start_x, float start_y, float end_x, float end_y);
  inline float p2inf_line_distance(float point_x, float point_y) const;
  inline float p2p_distance(float start_x, float start_y, float end_x, float end_y) const;

  line_point_t  __start;
  line_point_t  __end;
  float         __length;

  float         __nx;
  float         __ny;
  float         __normal_nx;
  float         __normal_ny;
  float         __line_normal_constant;
  float         __normal_line_normal_constant_start;
  float         __normal_line_normal_constant_end;
};

} //END: namespace Cox

#endif // __COX_LINE_2D_H__
