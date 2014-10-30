
/***************************************************************************
 *  field_lut.cpp - Lookup table for grid points on a soccer field
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


#include "FieldLut.h"
#include <SystemConfig.h>

#include <sys/stat.h>
#include <iostream>
#include <fstream>

namespace Cox
{


	time_t getModifyTime(const char * globalsPath)
	{
		struct stat file;
		stat(globalsPath, &file);
		//File does not exist or Permission Denied or the file has not been modified
		if( errno == 2 || errno == 13) {
			return 0;
		}
		else {
			return file.st_mtime;
		}
	}

  /** @class FieldLut field_lut.h
   * A lookup table for errors for given field position.
   * The table holds error and gradient for a given point on the field
   * to the next field line. The Sobel filter is used to calculate the
   * deviation in x and y direction.
   *
   * @fn void FieldLut::get_entry(float x, float y, field_lut_entry_t &entry)
   * Returns the nearest entry the the given position x/y. If the position is outside
   * the field the nearest entry on the field gets used and the distance to this entry
   * is added to the error.
   * @param x the x-coordinate of the position
   * @param y the y-coordiante of the position
   * @param entry a reference to the resulting entry
   *
   * @author Christof Hoppe
   * @author Christof Rath
   */

  /** Constructor.
   *
   * @param lines A list of field lines
   * @param circles A list of field circles (or parts thereof - center, corner, penalty)
   * @param resolution The resolution of the grid
   * @param border The border around the field
   */
  FieldLut::FieldLut(const field_lines_t &lines, const field_circles_t &circles, float resolution, float border)
  {
    __min_x = 1e50;
    __max_x = -1e50;
    __min_y = 1e50;
    __max_y = -1e50;
    __lut = NULL;
    __resolution = resolution;
    create_lut(lines, circles, border);
  }


  /** Destructor */
  FieldLut::~FieldLut()
  {
    delete [] __lut;
  }


  /** Lookup table creator.
   *
   * @param lines A list of field lines
   * @param circles A list of field circles (or parts thereof - center, corner, penalty)
   * @param border The border around the field
   */
  void FieldLut::create_lut(const field_lines_t &lines, const field_circles_t &circles, float border)
  {
    field_lines_t::const_iterator it = lines.begin();
    for (; it != lines.end(); ++it)
    {
      Line2D line = *it;
      __max_x = std::max(__max_x, line.get_start().x);
      __max_y = std::max(__max_y, line.get_start().y);
      __min_x = std::min(__min_x, line.get_start().x);
      __min_y = std::min(__min_y, line.get_start().y);

      __max_x = std::max(__max_x, line.get_end().x);
      __max_y = std::max(__max_y, line.get_end().y);
      __min_x = std::min(__min_x, line.get_end().x);
      __min_y = std::min(__min_y, line.get_end().y);
    }

    __min_x -= border;
    __max_x += border;
    __min_y -= border;
    __max_y += border;


    __entries_x = std::floor((__max_x - __min_x) / __resolution) + 1;
    __entries_y = std::floor((__max_y - __min_y) / __resolution) + 1;

    __lut = new field_lut_entry_t[__entries_x * __entries_y];


    supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
    std::string globalsfn = sc->getConfigPath() + "/Globals.conf";
    std::string LUfn = sc->getConfigPath() + "/CoxGradientLut.dat";

	if(true || getModifyTime(globalsfn.c_str()) > getModifyTime(LUfn.c_str())) {
		std::cout << "Regenerating GradientLookUp " << globalsfn.c_str() << " Modificationdates: " << getModifyTime(globalsfn.c_str()) << "\t" << getModifyTime(LUfn.c_str()) << std::endl;
	    for (float y = __min_y; y <= __max_y; y += __resolution)
	    {
	      for (float x = __min_x; x <= __max_x; x += __resolution)
	      {
		float min_dist = 1e50;
		float dist = 0.;

		for (field_lines_t::const_iterator it = lines.begin(); it != lines.end(); it++) {
		  dist = it->p2line_distance(x, y);

		  if (dist < min_dist)
		  {
		    min_dist = dist;
		  }
		}

		// check distance to circles
		for (field_circles_t::const_iterator it = circles.begin();
		        it != circles.end(); ++it)
		{
		  dist = it->get_distance(x,y);

		  if (dist < min_dist)
		  {
		    min_dist = dist;
		  }
		}

		__lut[get_index(x,y)].distance = min_dist;
	      }
	    }
	    calculate_gradient();
	    write_lutBinary(LUfn.c_str());
	} else {
		std::cout << "Loading GradientLookUp " << globalsfn.c_str() << " Modificationdates: " << getModifyTime(globalsfn.c_str()) << "\t" << getModifyTime("CoxGradientLut.dat") << std::endl;
		read_lutBinary(LUfn.c_str());
	}
	//read_lutBinary("CoxGradientLut.dat");
//    write_lutBinary("CoxGradientLut.dat");
//    write_lut("CoxGradientLut.txt");
  }


  /** Calculates the gradient of each point of the grid to the nearest line.
   * A Sobel filter is used.
   */
  void FieldLut::calculate_gradient()
  {
    unsigned int y0 = 0; //pointer to the row n-1
    unsigned int y1 = __entries_x; //pointer to the row n
    unsigned int y2 = 2 * y1; //pointer to the row n+1

    for (unsigned int y = 1; y < __entries_y -1; y++)
    {
      for (unsigned int x = 1; x < __entries_x -1 ; x++ )
      {
        // Calculate sobel operator
        float t_l = __lut[ y0 + (x - 1) ].distance; //Top left
        float c_l = __lut[ y1 + (x - 1) ].distance; //Center left
        float b_l = __lut[ y2 + (x - 1) ].distance; //Bottom left
        float t_c = __lut[ y0 + (x - 0) ].distance;
        float b_c = __lut[ y2 + (x - 0) ].distance;
        float t_r = __lut[ y0 + (x + 1) ].distance;
        float c_r = __lut[ y1 + (x + 1) ].distance;
        float b_r = __lut[ y2 + (x + 1) ].distance;

        __lut[y1 + x].dx = (t_l + 2.f * c_l + b_l - t_r - 2.f * c_r - b_r)/4.f;
        __lut[y1 + x].dy = (t_l + 2.f * t_c + t_r - b_l - 2.f * b_c - b_r)/4.f;
      }

      y0 = y1;
      y1 = y2;
      y2 += __entries_x;
    }

    //Filling the border values
    for (unsigned int x = 1; x < __entries_x - 1 ; ++x) {
      //First row:
      __lut[x].dx = __lut[__entries_x + x].dx;
      __lut[x].dy = __lut[__entries_x + x].dy;

      //Last row:
      __lut[y1 + x].dx = __lut[y0 + x].dx;
      __lut[y1 + x].dy = __lut[y0 + x].dy;
    }

    for (unsigned int y = 0; y < __entries_y; ++y)
    {
      //First column:
      __lut[y * __entries_x].dx = __lut[y * __entries_x + 1].dx;
      __lut[y * __entries_x].dy = __lut[y * __entries_x + 1].dy;

      //Last column:
      __lut[(y + 1) * __entries_x - 1].dx = __lut[(y + 1) * __entries_x - 2].dx;
      __lut[(y + 1) * __entries_x - 1].dy = __lut[(y + 1) * __entries_x - 2].dy;
    }

  }


  /** Writes the lookup table to a file (csv format)
   *
   * @param filename of the output
   */
  void FieldLut::write_lut(std::string filename)
  {
    std::ofstream out_stream;
    out_stream.open(filename.c_str());

    for (int y =(int)__entries_y-1; y >= 0; y-- )
    {
      for (int x =(int) __entries_x-1; x >= 0; x--)
      {
        out_stream << __lut[ y * __entries_x + x ].distance << " " << __lut[ y * __entries_x + x ].dx << " " << __lut[ y * __entries_x + x ].dy << " ";
      }
      out_stream << std::endl;
    }

    out_stream.close();
  }

  void FieldLut::write_lutBinary(std::string filename)
  {
    std::ofstream out_stream;
    out_stream.open(filename.c_str(), std::ios::binary);

    for (int y = 0; y < __entries_y; y++)
    {
      for (int x = 0; x < __entries_x; x++)
      {
        //out_stream << __lut[ y * __entries_x + x ].distance << " " << __lut[ y * __entries_x + x ].dx << " " << __lut[ y * __entries_x + x ].dy << " ";
	out_stream.write((char*)&(__lut[ y * __entries_x + x ]), sizeof (field_lut_entry_t));
      }
    }

    out_stream.close();
  }

  void FieldLut::read_lutBinary(std::string filename)
  {
    std::ifstream stream;
    stream.open(filename.c_str(), std::ios::binary);
    field_lut_entry_t tmp;
    bool shown = false;
    for (int y = 0; y < __entries_y; y++)
    {
      for (int x = 0; x < __entries_x; x++)
      {

        //out_stream << __lut[ y * __entries_x + x ].distance << " " << __lut[ y * __entries_x + x ].dx << " " << __lut[ y * __entries_x + x ].dy << " ";
	stream.read((char*)&(__lut[ y * __entries_x + x ]), sizeof (field_lut_entry_t));
	//stream.read((char*)&(tmp), sizeof(field_lut_entry_t));
	//if((__lut[ y * __entries_x + x ].distance != tmp.distance) || (__lut[ y * __entries_x + x ].dx != tmp.dx) || (__lut[ y * __entries_x + x ].dy != tmp.dy)) if(!shown) {std::cout << y << " "<< x  << " Fehler!!!" << (int)__entries_y-1 << " " << (int) __entries_x-1 << " " << sizeof(field_lut_entry_t) << std::endl; shown=true;}
      }
    }

    stream.close();
  }
}
