
/***************************************************************************
 *  cox_localization.h - The Cox localization algorithm
 *
 *  Created:  Thu Mar 26 14:20:34 2009
 *  Copyright 2008 Michael Reip   <mike1802@sbox.tugraz.at>
 *            2009 Christof Hoppe <christof@zirren.de>
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


#ifndef __COX_COX_LOCALIZATION_H__
#define __COX_COX_LOCALIZATION_H__

#include "FieldLut.h"
#include "../global/CoxTypes.h"
#include <map>
#include "FootballField.h"

namespace Cox
{
  typedef std::map<float, field_pose_t> error_pose_map_t;

  class CoxLocalization
  {
    public:
      CoxLocalization(field_lines_t &lines, 
                                   field_circles_t &circles, 
                                   float resolution = 20.f, 
                                   float border = 1000.f, 
                                   float stepsize_x = 40.f,
                                   float stepsize_y = 40.f, 
                                   float stepsize_phi = 0.1f);

      void set_2nd_deviation_thresholds(float xy, float phi);
      virtual ~CoxLocalization();
      void get_pose(field_pose_t &pose, line_points_t &scan);
      void global_localization(field_pose_t &pose, line_points_t &scan);
      
      
    protected: /* Methods */
      field_lines_t getLines();
      void rprob(float_vector_t &pose,float_vector_t &gradient, float_vector_t &former_gradient, float_vector_t &stepsize);
      void transform_scan(const float_vector_t &pose);
      virtual void get_cumulative_error(float_vector_t &gradient, const float_vector_t &pose);

    protected: /* Members */
      line_points_t __abs_scan;
      const line_points_t *__rel_scan;
      FieldLut __lut;

      float_vector_t __stepsize;
      float __xy_2nd_threshold;
      float __phi_2nd_threshold;


      // parameter for error function
      float __c_square;

      unsigned int __max_iterations;
  };
}
#endif // __COX_COX_LOCALIZATION_H__
