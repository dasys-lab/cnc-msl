/*
 * $Id: BallHelper.cpp 1531 2006-08-01 21:36:57Z phbaer $
 *
 *
 * Copyright 2005,2006 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */


#include "Draw.hpp"

#include <iostream>
#include <SystemConfig.h>

using std::cout;
using std::endl;

Draw::Draw()
{
	cout << "Start Draw Constructor" << endl;
	
	castor::SystemConfigPtr sc = castor::SystemConfig::getInstance();
	castor::Configuration *vision3D	= (*sc)["Vision3D"];
	
	cout << "Image" << endl;
	cout << "\tWidth_";
	width	= vision3D->get<uint16_t>("Image", "Width", NULL);
	cout << "\tHeight_";
	height	= vision3D->get<uint16_t>("Image", "Height", NULL);
	
	cout << "End Draw Constructor" << endl;
}

Draw::~Draw()
{
	cout << "Destructor of Draw" << endl;
}


/*
** Draw a circle with the Bresenham - Algorithm
*/
void Draw::Circle(unsigned char *&dst, struct Circle &circ)
{
	uint16_t	x		= circ.x + width/2;
	uint16_t	y		= circ.y + height/2;
	uint16_t	radius	= circ.radius;
	
	int32_t	f		= 1 - radius;
	int32_t	ddF_x	= 0;
	int32_t	ddF_y	= -2 * radius;
	int32_t	currX	= 0;
	int32_t	currY	= radius;
	
	unsigned char intensity = 0;
	uint32_t mid = x + y * width;
	
	if( dst[mid+radius] < 128 )
		intensity = 255;
	
	if( radius < width/2 )
	{
	  dst[mid+radius]		= intensity;
	  dst[mid-radius]		= intensity;
	}
	if( radius < height/2 )
	{
	  dst[mid+radius*width]	= intensity;
	  dst[mid-radius*width]	= intensity;
	}
	
	while(currX<currY)
	{
		if(f >= 0)
		{
			currY--;
			ddF_y += 2;
			f += ddF_y;
		}
		currX++;
		ddF_x += 2;
		f += ddF_x + 1;
		
		if( (currX < width/2) && (currY < height/2) )
		{
		  dst[mid+currX+currY*width] = intensity;
		  dst[mid-currX+currY*width] = intensity;
		  dst[mid+currX-currY*width] = intensity;
		  dst[mid-currX-currY*width] = intensity;
		}
		if( (currX < height/2) && (currY < width/2) )
		{
		  dst[mid+currY+currX*width] = intensity;
		  dst[mid+currY-currX*width] = intensity;
		  dst[mid-currY+currX*width] = intensity;
		  dst[mid-currY-currX*width] = intensity;
		}
	}
}



void Draw::ScanLine(unsigned char * &dst)
{
	unsigned char * ptr = dst;
	
	uint16_t x;
	uint16_t y;
	uint32_t index;
	uint16_t * lines;
	uint32_t * linesOffsets;
	bool thick = false;
	
	// Inner lines
	lines 		= ScanLineHelper3D::getLinesInner();
	linesOffsets	= ScanLineHelper3D::getLinesInnerOffsets();
	
	// For each line
	for(uint16_t a = 0; a < ScanLineHelper3D::getNumberLines(); a++)
	{
		uint16_t * line = lines + linesOffsets[a];
		
		for(uint32_t j = linesOffsets[a]; j < linesOffsets[a+1]; j+=2)
		{
			x = *line++;
			y = *line++;
			
			index = y*width+x;
			ptr[index] = 255;
			if (thick)
			{
				ptr[index+1] = 255;
				ptr[index-1] = 255;
				ptr[index+width] = 255;
				ptr[index-width] = 255;
			}
		}
	}
	
	// Outer lines
	lines		= ScanLineHelper3D::getLinesOuter();
	linesOffsets	= ScanLineHelper3D::getLinesOuterOffsets();
	
	// For each line
	for(uint16_t a = 0; a < ScanLineHelper3D::getNumberLines(); a++)
	{
		uint16_t * line = lines + linesOffsets[a];
		
		for(uint32_t j = linesOffsets[a] + 2; j < linesOffsets[a+1]; j+=2)
		{
			x = *line++;
			y = *line++;
			
			index = y*width+x;
			ptr[index] = 255;
			if (thick)
			{
				ptr[index+1] = 255;
				ptr[index-1] = 255;
				ptr[index+width] = 255;
				ptr[index-width] = 255;
			}
		}
	}
}


