/*
 * $Id: FilterExtractImages.cpp 2142 2007-04-15 10:49:00Z jewollen $
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
#include "FilterExtractImages.hpp"

#include <iostream>
#include <math.h>
#include <string>
#include <fstream>
#include <SystemConfig.h>

using std::cout;
using std::endl;
using std::string;
using std::ifstream;


FilterExtractImages::FilterExtractImages()
{
	cout << "Start FilterExtractImages Constructor" << endl;
	
	castor::SystemConfigPtr sc = castor::SystemConfig::getInstance();
	castor::Configuration *vision3D = (*sc)["Vision3D"];
	
	lTsize	= 256*256;
	lTUVYsize	= 256*256*256;
	
	cout << "Image" << endl;
	cout << "\tWidth_";
	width	= vision3D->get<uint16_t>("Image", "Width", NULL);
	cout << "\tHeight_";
	height	= vision3D->get<uint16_t>("Image", "Height", NULL);

	/// Get configs
	cout << "ScanLines" << endl;
	cout << "\tInnerRadiusStart_";
	iRadiusStart	= vision3D->get<uint16_t>("ScanLines", "InnerRadiusStart", NULL);
	cout << "\tInnerRadiusEnd_";
	iRadiusEnd	= vision3D->get<uint16_t>("ScanLines", "InnerRadiusEnd", NULL);
	cout << "\tOuterRadiusStart_";
	oRadiusStart	= vision3D->get<uint16_t>("ScanLines", "OuterRadiusStart", NULL);
	cout << "\tOuterRadiusEnd_";
	oRadiusEnd	= vision3D->get<uint16_t>("ScanLines", "OuterRadiusEnd", NULL);
	
	/// Memory Alloc
	grayImage	= new unsigned char [width*height];
	uvImage	= new unsigned char [width*height*2];
	
	/// Setup lookupTableUVY
	lookupTableUVY	= new unsigned char [lTUVYsize];
	if(lookupTableUVY == NULL)	cout << "lookupTableuvy = null!!!" << endl;
	
	for(int y=0; y<256; y++)
	{
		for(int u=0; u<256; u++)
		{
			for(int v=0; v<256; v++)
			{
				if((int)v-(int)u - ((int)y/2)>0 && (int)v-(int)u - ((int)y/2)<255)
					lookupTableUVY[y*256*256 + u*256 + v] = (int)v-(int)u - ((int)y/2);
				else
					lookupTableUVY[y*256*256 + u*256 + v]=0;
			}
		}
	}
	
	/// Setup lookupTable
	lookupTable	= new unsigned char [lTsize];
	if(lookupTable == NULL)	cout << "lookupTable = null!!!" << endl;
	
	init();
	
	string filename = string(sc->getConfigPath())+string("/LookupTable");
	ifstream ifs(filename.c_str());
	
	double t;
	
	for(int u=0; u<256; u++)
	{
		for(int v=255; v>=0; v--)
		{
			ifs >> t;
			lookupTable[u*256+v] = (unsigned char) t;
		}
	}
	
	ifs.close();

	cout << "End FilterExtractImages Constructor" << endl;
}


FilterExtractImages::~FilterExtractImages()
{
	cout << "Destructor of FilterExtractImages" << endl;;
	
	if(uvImage != NULL)			delete[] uvImage;
	if(grayImage != NULL)		delete[] grayImage;
	if(lookupTable != NULL)		delete[] lookupTable;
	if(lookupTableUVY != NULL)	delete[] lookupTableUVY;
}


bool FilterExtractImages::setLookupTableValue( const uint32_t index, const unsigned char value )
{
	if(index>=lTsize)
		return 0;
	
	lookupTable[index] = value;
	return 1;
}

bool FilterExtractImages::setLookupTableUVYValue(uint32_t const index, unsigned char const value)
{
	if(index>=lTUVYsize)
		return 0;
	
	lookupTableUVY[index] = value;
	return 1;
}

unsigned char FilterExtractImages::getLookupTableValue(uint32_t const index) const
{
	if(index>=lTsize)
		throw 0;	// TODO
	return lookupTable[index];
}

unsigned char FilterExtractImages::getLookupTableUVYValue(uint32_t const index) const
{
	if(index>=lTUVYsize)
		throw 0; // TODO
	return lookupTableUVY[index];
}


void FilterExtractImages::process(unsigned char * &src, unsigned char * &grayImage_, unsigned char * &uvImage_)
{
	unsigned char *grayPointer	= grayImage;
	unsigned char *uvPointer		= uvImage;
	unsigned char *ptr			= src;
	
	register unsigned char u = 0;
	register unsigned char y = 0;
	register unsigned char v = 0;
	register int color;
	
	
	for(uint16_t i = 0; i<height; i++)
	{
		for(uint16_t j = 0; j<width; j++)
		{
			/* Get uv */
			if(j%2==0)	u = *ptr++;
			else		v = *ptr++;
			
			/* Calculate color */
			color = u*256 + v;
			
			/* Store uv */
			if(y <= 220)	*uvPointer++ = lookupTable[color];
			else			*uvPointer++ = 0;
			
			/* Get gray */
			y = *ptr++;
			
			/* Store gray */
			*grayPointer++ = y;
		}
	}
	grayImage_	= grayImage;
	uvImage_	= uvImage;
}


void FilterExtractImages::init()
{
	int center = 128;
	double angle = atan2(150 - center, 70 - center);
	
	for(int u = 0; u < 256; u++){
		for(int v = 0; v < 256; v ++)
		{
			int value = (int) lrint(((cos(angle)*(u-center) + sin(angle)*(v-center)) + 128)*2.0);
			double diffAngle = fabs(atan2(v-center, u-center) - angle);
			double distance = sqrt((v-center)*(v-center) + (u-center)*(u-center))/128.0;
			if(diffAngle > M_PI)
				diffAngle = fabs(diffAngle - 2.0*M_PI);
			value -= (int) lrint(7.5*diffAngle*180.0/M_PI*(0.3*distance + 1.0));
			
			int value2 = 0;//(int) lrint( (((double)(pow(v, 1.8)-u))/64.0)-0.1*diffAngle*(180.0/M_PI) );
			
			if(value2 > value)
				value = value2;
			
			if(value < 0)
				value = 0;
			if(value > 255)
				value = 255;
			
			lookupTable[u*256 + v] = (unsigned char) value; 
		}
	}
}

