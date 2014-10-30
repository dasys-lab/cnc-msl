
/***************************************************************************
 *  cox_localization.cpp - The Cox localization algorithm
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


#include "CoxLocalization.h"
#include <iostream>
#include <cstdlib>
#include <utility>

namespace Cox
{
  /** @class CoxLocalization cox_localization.h <nao_utils/cox/cox_localization.h>
   * Implements the Cox localization algorithm
   *
   * @author Michael Reip
   * @author Christof Hoppe
   * @author Christof Rath
   */

  /** @var line_points_t CoxLocalization::__abs_scan
   * The list of line points transformed to absolute coordinates
   */
  /** @var const line_points_t *CoxLocalization::__rel_scan
   * The list of line points relative to the initial pose
   */
  /** @var FieldLut &CoxLocalization::__lut
   * The field lookup table @see Cox::FieldLut
   */
  /** @var float_vector_t CoxLocalization::__stepsize
   * The initial stepsizes for x/y/phi. The actual stepsizes gets changed
   * during each iteration by rprop
   */
  /** @var float CoxLocalization::__c_square
   * Parameter for error function set according to a paper to 250²
   */
  /** @var unsigned int CoxLocalization::__max_iterations
   * The number of iterations to find the new pose estimate
   */

  /** Constructor.
   *
   * @param lut The field lookup table
   * @param stepsize_x The initial stepsize in x-direction
   * @param stepsize_y The initial stepsize in y-direction
   * @param stepsize_phi The initial stepsize for the orientation
   */
  CoxLocalization::CoxLocalization(field_lines_t &lines,
                                   field_circles_t &circles,
                                   float resolution,
                                   float border,
                                   float stepsize_x,
                                   float stepsize_y,
                                   float stepsize_phi):
      __lut(lines, circles, resolution, border)
  {
    __stepsize.push_back(stepsize_x);
    __stepsize.push_back(stepsize_y);
    __stepsize.push_back(stepsize_phi);
    __c_square = 250 * 250;
    __max_iterations = 10;
    __xy_2nd_threshold = 0.0;
    __phi_2nd_threshold = 0.0;
  }


  /** Dummy destructor */
  CoxLocalization::~CoxLocalization()
  {
  }


  /** Transforms a scan of relative coordinate to an absolute position
   *
   * @param pose The expected/estimated pose of the scan
   */
  void CoxLocalization::transform_scan(const float_vector_t &pose)
  {

    float cosinus = cosf(pose[2]);
    float sinus = sinf(pose[2]);

    __abs_scan.clear();
    float x, y;
    for (line_points_t::const_iterator it = __rel_scan->begin(); it != __rel_scan->end(); ++it)
    {
      x = (cosinus * it->x) - (sinus * it->y) + pose[0];
      y = (sinus * it->x) + (cosinus * it->y) + pose[1];
      __abs_scan.push_back(line_point_t(x, y));
    }
  }

  /** Calculates the error over all points of the transformed scan
   *
   * @param gradient the resulting gradient
   * @param pose the current pose (expected/estimated)
   */
  void CoxLocalization::get_cumulative_error(float_vector_t &gradient, const float_vector_t &pose)
  {
    float __c = 250;
    float sinphi = sinf(pose[2]);
    float cosphi = cosf(pose[2]);

    gradient.clear();
    float distance = 0.f;
    float dx = 0.f;
    float dy = 0.f;
    float dphi = 0.f;
    // Hessian diagonal elements (2. derivation)
    float hx = 0.f;
    float hy = 0.f;
    float hphi = 0.f;
    // variables containing information about 2. derivative
    float derr  = 0.f; // Error 1. derivative
    float dderr = 1.f / __c_square;
    float dposdphi_x = 0.f;
    float dposdphi_y = 0.f;
    float dposdphi_x2 = 0.f;
    float dposdphi_y2 = 0.f;

    float t1 = 0.f;
    float t2 = 0.f;

    field_lut_entry_t entry;


    float ef = 0.f;
    float derrddist = 0.f;

    //Global coords
    float x = 0.f;
    float y = 0.f;
    //Relative coords
    float r = 0.f;
    float s = 0.f;

    line_points_t::iterator abs_it = __abs_scan.begin();
    for (line_points_t::const_iterator rel_it = __rel_scan->begin(); rel_it != __rel_scan->end(); ++rel_it, ++abs_it)
    {
      x = abs_it->x;
      y = abs_it->y;

      r = rel_it->x;
      s = rel_it->y;
      __lut.get_entry(x,y, entry);
      distance += entry.distance;

      //now calculate the derivation stuff
      ef = __c_square + entry.distance * entry.distance;
      derrddist = (2 * __c_square * entry.distance) /(ef * ef);
      dx += derrddist * entry.dx;
      dy += derrddist * entry.dy;
      dphi += derrddist * (entry.dx * (-sinphi * r - cosphi * s ) + entry.dy * (cosphi * r - sinphi * s));
      // 2. derviative
      if (entry.distance < 2 * __c ) {
        derr = entry.distance / __c_square;
        dposdphi_x = -sinphi * r - cosphi * s;
        dposdphi_y = cosphi * r - sinphi *s;
        dposdphi_x2 = -cosphi * r + sinphi * s;
        dposdphi_y2 = -sinphi * r - cosphi * s;

        hx += dderr * entry.dx * entry.dx;
        hy += dderr * entry.dy * entry.dy;

        t1 = entry.dx * dposdphi_x + entry.dy * dposdphi_y;
        t2 = entry.dx * dposdphi_x2 + entry.dy * dposdphi_y2;

        hphi += dderr * t1 * t1 + derr * t2;
      }
    }

    gradient.push_back(dx);
    gradient.push_back(dy);
    gradient.push_back(dphi);
    gradient.push_back(hx);
    gradient.push_back(hy);
    gradient.push_back(hphi);

  }


  /** Estimates the current position
   *
   * @param pose the initial and than estimated pose
   * @param rel_scan the scan relative to the initial position
   */
  void CoxLocalization::get_pose(field_pose_t &pose, line_points_t &rel_scan)
  {
    // if no entries in the relative scan
    if (!rel_scan.size()) return;

    float_vector_t cur_pose;
    cur_pose.push_back(pose.x);
    cur_pose.push_back(pose.y);
    cur_pose.push_back(pose.phi);

    __rel_scan = &rel_scan;
    float_vector_t gradient;

    float_vector_t stepsize = __stepsize;
    float_vector_t former_gradient(3, 0.f);

    // delete all elements that are outside the field
    transform_scan(cur_pose);
    line_points_t::const_iterator abs_iter = __abs_scan.begin();
    line_points_t::iterator rel_iter = rel_scan.begin();
    line_points_t::iterator del_iter = rel_scan.begin();

    int deleted_lines = 0;

    while (abs_iter != __abs_scan.end())
    {
      rel_iter++;
      if (__lut.outside_field(abs_iter->x, abs_iter->y))
      {
        rel_scan.erase(del_iter);
        deleted_lines++;
      }
      del_iter = rel_iter;
      abs_iter++;
    }


    for (unsigned int iter = 0; iter < __max_iterations; iter++)
    {
      transform_scan(cur_pose);
      get_cumulative_error(gradient, cur_pose);
      rprob(cur_pose, gradient, former_gradient, stepsize);
      
    }
    float middle = 0.0;

  //  for(int i =0; i<gradient.size(); i++) { std::cout<<"NewLoc gradient"<<i<<": "<<gradient[i]<<std::endl; middle += gradient[i]; }
    //std::cout<<"NewLoc gradmiddle: "<<middle/gradient.size()<<std::endl;
//	if (middle !=0 )
//	{
	    pose.x = cur_pose[0];
	    pose.y = cur_pose[1];
	    pose.phi = cur_pose[2];
//	}
//	else
	{
//		std::cout<<"NewLoc globalloc"<<std::endl;
//		global_localization(pose, rel_scan);
	}


  }

  void CoxLocalization::global_localization(field_pose_t &pose_init, line_points_t &rel_scan)
  {
    std::cout<<"NewLoc globalloc"<<std::endl;
    float_vector_t error;
    float_vector_t pose_vector;
    error_pose_map_t error_pose_map;

    for (unsigned int iterations = 0; iterations < 1000; iterations++)
    {
      pose_vector.clear();
      int x = (rand() % ( abs( __lut.get_max_x() - __lut.get_min_x() ) )) - abs(__lut.get_min_x()) ;
      int y = (rand() % ( abs( __lut.get_max_y() - __lut.get_min_y() ) )) - abs(__lut.get_min_y()) ;
//      float phi = (float((rand() % 2*314)) - 314) / 100;
      float phi = pose_init.phi;
      pose_vector.push_back(x);
      pose_vector.push_back(y);
      pose_vector.push_back(phi);

      field_pose_t pose(x, y, phi);
      get_pose(pose, rel_scan);
      get_cumulative_error(error, pose_vector);
      error_pose_map.insert(std::make_pair(error[3], pose));
    }

    error_pose_map_t::iterator it = error_pose_map.begin();
    error_pose_map_t second_iteration;

    for (unsigned int i = 0; i < 20; i++)
    {
      field_pose_t pose = it->second;
      pose_vector.clear();
      get_pose(pose, rel_scan);
      pose_vector.push_back(pose.x);
      pose_vector.push_back(pose.y);
      pose_vector.push_back(pose.phi);
      get_cumulative_error(error, pose_vector);
      second_iteration.insert(std::make_pair(error[3], pose));
      it++;
    }

    it = second_iteration.begin();
    pose_init.x = (it->second).x;
    pose_init.y = (it->second).y;
    pose_init.phi = (it->second).phi;

  }

  /** Calculates the next estimate for the pose using rprob
   *
   * @param pose the current/next pose
   * @param gradient the gradient from the error function
   * @param former_gradient the gradient of the last cycle
   * @param stepsize the current stepsize
   */
  void CoxLocalization::rprob(float_vector_t &pose, float_vector_t &gradient, float_vector_t &former_gradient, float_vector_t &stepsize)
  {
    for (unsigned int i = 0; i < 3; ++i) {
      // update fuer jeden Parameter vornehmen
      if (gradient[i] == 0) former_gradient[i] = 0;
      else if ((i < 2) && fabsf(gradient[i + 3]) < __xy_2nd_threshold) {
        gradient[i] = former_gradient[i] = 0;
        std::cout << "NewLoc Not updating dimension ";
        if (i == 0)
          std::cout << "X ";
        else
          std::cout << "Y ";
        std::cout <<"(" << fabsf(gradient[i + 3]) <<"<" << __xy_2nd_threshold << ")" << std::endl;
      }
      else {
	// Schrittweitenanpassung
        if (gradient[i] * former_gradient[i] > 0) stepsize[i] *= 1.2f;
        else if (gradient[i] * former_gradient[i] < 0) stepsize[i] *= 0.5f;

        former_gradient[i] = gradient[i];

//	std::cout<<"NewLoc Stepsize "<<stepsize[i]<<std::endl;
	// Anpassung der Parameter
        if (gradient[i] > 0) pose[i] += stepsize[i];
        else if (gradient[i] < 0) pose[i] -= stepsize[i];
      }
    }
  }

  /** Threshold setter for the 2nd deviation parameters
   * used to avoid optimization, when not possible (e.g. along a single line, ...)
   * @param xy The parameter in x- and y-direction
   * @param phi The parameter for the rotation
   */
  void CoxLocalization::set_2nd_deviation_thresholds(float xy, float phi)
  {
    __xy_2nd_threshold = xy;
    __phi_2nd_threshold = phi;
  }
  

  

} //END: namespace Cox
