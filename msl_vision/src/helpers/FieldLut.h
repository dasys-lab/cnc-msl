
/***************************************************************************
 *  field_lut.h - Lookup table for grid points on a soccer field
 *
 *  Created:  Fri Mar 27 15:20:34 2009
 *  Copyright 2009 Christof Hoppe <christof@zirren.de>
 *                 Christof Rath  <christof.rath@gmail.com>
 *
 *  $Id$
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


#ifndef __COX_FIELDLUT_H__
#define __COX_FIELDLUT_H__

#include "../global/CoxTypes.h"
#include "Line2D.h"
#include <cstdio>
namespace Cox
{
  class FieldLut
  {

    public:
      FieldLut(const field_lines_t &lines, const field_circles_t &circles, float resolution, float border);
      ~FieldLut();
      
      inline bool outside_field(float x, float y)
      {
        if((x < __min_x) || (x > __max_x) ||
           (y < __min_y) || (y > __max_y))
        {
          return true;
        }
       else
       {
         return false;
       }
      }

      inline void get_entry(float x, float y, field_lut_entry_t &entry)
      {
        unsigned int index = get_index(x,y);

        if (index == (unsigned int)-1)
        {
          float b_x = x < __min_x ? __min_x : (x > __max_x ? __max_x : x);
          float b_y = y < __min_y ? __min_y : (y > __max_y ? __max_y : y);
          index = get_index(b_x, b_y);
          field_lut_entry_t e = __lut[index];
          entry.distance = e.distance + sqrt((b_x - x) * (b_x - x) + (b_y - y) * (b_y - y));
          entry.dx = e.dx;
          entry.dy = e.dy;
        }
        else
        {
          field_lut_entry_t e = __lut[index];
          entry.distance = e.distance;
          entry.dx = e.dx;
          entry.dy = e.dy;
        }
      }

      float get_max_x() {return __max_x;};
      float get_max_y() {return __max_y;};
      float get_min_x() {return __min_x;};
      float get_min_y() {return __min_y;};

    private:
      void create_lut(const field_lines_t &lines, const field_circles_t &circles, float border);
      void calculate_gradient();

      void write_lut(std::string filename);
      void write_lutBinary(std::string filename);
      void read_lutBinary(std::string filename);

      inline unsigned int get_index(float x, float y)
      {
        if ((x < __min_x) || (x > __max_x) || (y < __min_y) || (y > __max_y))
        {
          return -1;
        }

        unsigned int id_x = lrint((x - __min_x) / __resolution);
        unsigned int id_y = lrint((y - __min_y) / __resolution);
        return id_y * __entries_x + id_x;
      }

      unsigned int __entries_x;
      unsigned int __entries_y;

      float __max_x;
      float __max_y;
      float __min_x;
      float __min_y;
      float __resolution;

      field_lut_entry_t* __lut;
  };
}

#endif //__COX_FIELDLUT_H__
