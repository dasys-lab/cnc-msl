/*
 * $Id: FilterTitanBraces.cpp 1935 2007-03-19 19:50:12Z phbaer $
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
#include "FilterTitanBraces.hpp"

#include <algorithm>
#include <math.h>

FilterTitanBraces::FilterTitanBraces(ScanCircleHelper helper)
{
	SystemConfigPtr sc = SystemConfig::getInstance();
	Configuration *vision3D	= (*sc)["Vision3D"];
	Configuration *loc		= (*sc)["Localization3D"];
	
	width	= vision3D->get<uint16_t>("Image", "Width", NULL);
	height	= vision3D->get<uint16_t>("Image", "Height", NULL);
	
	centerX	= width/2;
	centerY	= height/2;
	
	CirclePointsThreshold	= loc->get<uint16_t>("Localization", "CirclePointsThreshold", NULL);
	CirclePointsJump		= loc->get<uint16_t>("Localization", "CirclePointsJump", NULL);
	MinBracesWidth			= loc->get<uint16_t>("Localization", "MinBracesWidth", NULL);
	MaxBracesWidth			= loc->get<uint16_t>("Localization", "MaxBracesWidth", NULL);
	floorBrightness		= loc->get<uint16_t>("Localization", "floorBrightness", NULL);
	
	circles		= helper.getCircles();
	circlesOffset	= helper.getCircleOffsets();
	maxPoints		= helper.getMaxPoints();
	nCircles		= helper.getNumberCircles();
}



void FilterTitanBraces::process(unsigned char * src)
{
	register uint16_t x;
	register uint16_t y;
	
	std::vector<uint16_t> circlePoints;
	circlePoints.clear();
	
	CirclePoints1.clear();
	CirclePoints2.clear();
	CirclePoints3.clear();
	
	/// Finding titan braces due to dark/light/dark jumps over scanCircles.
	for(uint16_t i=0; i<nCircles; i++)
	{
		uint32_t start = 0;
		uint32_t end = 0;
		
		// StartPoint
		uint16_t * circle = circles + circlesOffset[i];
		x = *circle++;
		y = *circle++;
		
		unsigned char grayOld = src[x+y*width];
		
		uint32_t points = abs(circlesOffset[i]/2);
		
		for(uint32_t j=1; j<points; j++)
		{
			x = *circle++;
			y = *circle++;
			
			unsigned char grayNew = src[x+y*width];
			
			if(grayNew<(grayOld-CirclePointsJump) && grayNew<CirclePointsThreshold )
			{
				start = j;
				end = j;
			}
			
			if(grayNew<(grayOld+CirclePointsJump) && grayNew>floorBrightness)
			{
				end = j;
			}
			
 			if( (start>0) && (end-start)>MinBracesWidth)// && (end-start)<(MaxBracesWidth) )
			{
				uint32_t index = circlesOffset[i] + end+start;	// = (end+start)/2 * 2
				if( index%2!=0 )
					index--;
				int16_t indX = circles[index];
				int16_t indY = circles[index+1];
				
				// Mark line in image
				if( src[indX+indY*width] < 60 )
					src[indX+indY*width] = 255;
				else
					src[indX+indY*width] = 0;
				
				// Move zero to the middle.
				indX -= centerX;
				indY -= centerY;
				
				circlePoints.push_back((uint16_t)(sqrt(indX*indX+indY*indY)));
				
				start = 0;
				end = 0;
			}
			grayOld = grayNew;
		}
		circlePoints.clear();
	}
	printf("FilterTitanBraces - Number of CirclePoints: %d\n", (int)circlePoints.size());
}



FilterTitanBraces::~FilterTitanBraces()
{
}

