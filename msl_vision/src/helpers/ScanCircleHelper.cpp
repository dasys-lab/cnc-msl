/*
 * $Id: ScanCircleHelper.cpp 1987 2007-04-09 16:58:10Z rreichle $
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
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "ScanCircleHelper.hpp"


ScanCircleHelper::ScanCircleHelper()
{
	this->sc = SystemConfig::getInstance();
	Configuration *vision3D = (*this->sc)["Vision3D"];
	
	width	= vision3D->get<uint16_t>("Image", "Width", NULL);
	height	= vision3D->get<uint16_t>("Image", "Height", NULL);
	centerX	= width/2;
	centerY	= height/2;
	
	iRadiusStart	= vision3D->get<uint16_t>("ScanLines", "InnerRadiusStart", NULL);
	iRadiusEnd	= vision3D->get<uint16_t>("ScanLines", "InnerRadiusEnd", NULL);
	oRadiusStart	= height/2;	//vision3D->get<uint16_t>("ScanLines", "OuterRadiusStart", NULL);
	oRadiusEnd	= vision3D->get<uint16_t>("ScanLines", "OuterRadiusEnd", NULL);
	circleOffset	= vision3D->get<uint16_t>("ScanLines", "CircleOffset", NULL);
	
	numberOfCircles	= floor((oRadiusStart-iRadiusStart)/circleOffset);
	maxPoints			= ceil(height*M_PI);
	
 	circles		= new uint16_t [maxPoints*numberOfCircles*2];
	circleOffsets	= new uint32_t [numberOfCircles];
	
	init();
}



void ScanCircleHelper::init( )
{
	uint32_t currOffset = 0;
	
	uint16_t * circlePtr = circles;
	
	for(uint16_t a=0; a<numberOfCircles; a++)
	{
		circleOffsets[a] = currOffset;
		
		uint16_t radius = iRadiusStart + a*circleOffset;
		
		if( radius>= iRadiusEnd && radius<=oRadiusEnd )
			continue;
		
		int32_t currX = 0;
		int32_t currY = 0;
		
		currX = centerX + radius;
		currY = centerY;
		
		// First segment
		while(currX >= centerX && currY >= centerY - radius)
		{
			double m = 2.0*radius*radius;
			int32_t indX = -1;
			int32_t indY = -1;
			int32_t cX = 0;
			int32_t cY = 0;
			
			for(int8_t i=-1; i<=0; i++)
			{
				for(int8_t j=-1; j<=0; j++)
				{
					if( i!=0 || j!=0 )
					{
						cX = currX + i;
						cY = currY + j;
						
						double f = fabs((cX - centerX)*(cX - centerX) + (cY - centerY)*(cY - centerY) - radius*radius*1.0);
						
						if(f<m)
						{
							m = f;
							indX = cX;
							indY = cY;
						}
					}
				}
			}
			
			currX = indX;
			currY = indY;
			
			if(currX < 0 || currX >= width || currY < 0 || currY >= height)
			{
				std::cout << "Circles out of bounds!" << std::endl;
				continue;
			}
			
			*circlePtr++ = currX;
			*circlePtr++ = currY;
			currOffset++;
			
			//printf("Circles currX : %d currY : %d\n", currX, currY);
		}
		
		// Second segment
		while(currX >= centerX - radius && currY <= centerY)
		{
			double m = 2.0*radius*radius;
			int32_t indX = -1;
			int32_t indY = -1;
			int32_t cX = 0;
			int32_t cY = 0;
			
			for(int8_t i=-1; i<=0; i++)
			{
				for(int8_t j=0; j<=1; j++)
				{
					if( i!=0 || j!=0 )
					{
						cX = currX + i;
						cY = currY + j;
						
						double f = fabs((cX - centerX)*(cX - centerX) + (cY - centerY)*(cY - centerY) - radius*radius*1.0);
						
						if(f<m)
						{
							m = f;
							indX = cX;
							indY = cY;
						}
					}
				}
			}
			
			currX = indX;
			currY = indY;
			
			if(currX < 0 || currX >= width || currY < 0 || currY >= height)
			{
				std::cout << "Circles out of bounds!" << std::endl;
				continue;
			}
			
			*circlePtr++ = currX;
			*circlePtr++ = currY;
			currOffset++;
		}
		
		// Third segment
		while(currX <= centerX && currY <= centerY + radius)
		{
			double m = 2.0*radius*radius;
			int32_t indX = -1;
			int32_t indY = -1;
			int32_t cX = 0;
			int32_t cY = 0;
			
			for(int8_t i=0; i<=1; i++)
			{
				for(int8_t j=0; j<=1; j++)
				{
					if( i!=0 || j!=0 )
					{
						cX = currX + i;
						cY = currY + j;
						
						double f = fabs((cX - centerX)*(cX - centerX) + (cY - centerY)*(cY - centerY) - radius*radius*1.0);
						
						if(f<m)
						{
							m = f;
							indX = cX;
							indY = cY;
						}
					}
				}
			}
			
			currX = indX;
			currY = indY;
			
			if(currX < 0 || currX >= width || currY < 0 || currY >= height)
			{
				std::cout << "Circles out of bounds!" << std::endl;
				continue;
			}
			
			*circlePtr++ = currX;
			*circlePtr++ = currY;
			currOffset++;
		}
		
		// Last segment
		while(currX <= centerX + radius && currY >= centerY)
		{
			double m = 2.0*radius*radius;
			int32_t indX = -1;
			int32_t indY = -1;
			int32_t cX = 0;
			int32_t cY = 0;
			
			for(int8_t i=0; i<=1; i++)
			{
				for(int8_t j=-1; j<=0; j++)
				{
					if( i!=0 || j!=0 )
					{
						cX = currX + i;
						cY = currY + j;
						
						double f = fabs((cX - centerX)*(cX - centerX) + (cY - centerY)*(cY - centerY) - radius*radius*1.0);
						
						if(f<m)
						{
							m = f;
							indX = cX;
							indY = cY;
						}
					}
				}
			}
			
			currX = indX;
			currY = indY;
			
			if(currX < 0 || currX >= width || currY < 0 || currY >= height)
			{
				std::cout << "Circles out of bounds!" << std::endl;
				continue;
			}
			
			*circlePtr++ = currX;
			*circlePtr++ = currY;
			currOffset++;
		}
	}
	std::cout << "Circles currOffset = " << currOffset << std::endl;
	circleOffsets[numberOfCircles] = currOffset;
}

uint16_t ScanCircleHelper::getMaxPoints() const		{	return maxPoints;	}
uint16_t * ScanCircleHelper::getCircles()const 		{	return circles;	}
uint32_t * ScanCircleHelper::getCircleOffsets() const	{	return circleOffsets;	}
uint16_t ScanCircleHelper::getNumberCircles() const	{	return numberOfCircles;	}


ScanCircleHelper::~ScanCircleHelper()
{
	std::cout << "Destructor of ScanCircleHelper" << std::endl;
	
	if(circles != NULL)		delete[] circles;
	if(circleOffset != NULL)	delete[] circleOffsets;
}