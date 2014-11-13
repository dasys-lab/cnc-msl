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
#include "FilterExtractPanno.hpp"

#include <string>
#include <fstream>


FilterExtractPanno::FilterExtractPanno(struct ImageSize &innerSize, struct ImageSize &outerSize, struct ImageSize &pannoSize)
{
	cout << "Start FilterExtractPanno Constructor" << endl;
	
	this->sc = SystemConfig::getInstance();
	Configuration *vision3D = (*this->sc)["Vision3D"];
	
	cout << "Image" << endl;
	cout << "\tWidth_";
	width	= vision3D->get<uint16_t>("Image", "Width", NULL);
	cout << "\tHeight_";
	height	= vision3D->get<uint16_t>("Image", "Height", NULL);
	cout << "\tPannoWidth_";
	pWidth		= vision3D->get<uint16_t>("Image", "Panno", "Width", NULL);
	
	
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
	
	pHeightInner	= iRadiusEnd - iRadiusStart;
	pHeightOuter	= oRadiusStart - oRadiusEnd;
	pHeight		= pHeightInner+pHeightOuter;
	
	innerSize.width	= pWidth;
	innerSize.height	= pHeightInner;
	outerSize.width	= pWidth;
	outerSize.height	= pHeightOuter;
	pannoSize.width	= pWidth;
	pannoSize.height	= pHeight;
	
	// Setup display
	pannoDisplay	= new XVDisplay(pWidth, pHeight, XV_UYVY);
	
	/// Memory Alloc
	pannoImage		= new unsigned char [pWidth*pHeight];
	innerPanno		= new unsigned char [pWidth*pHeightInner];
	innerPannoTrafo	= new uint16_t [pWidth*pHeightInner*2];
	outerPanno		= new unsigned char [pWidth*pHeightOuter];
	outerPannoTrafo	= new uint16_t [pWidth*pHeightOuter*2];
	
	
	/// Generate InnerTrafo
	uint16_t * pointer = innerPannoTrafo;
	for(int16_t py=(pHeightInner-1); py>=0; py--)
	{
		for(uint16_t px=0; px<pWidth; px++)
		{
			uint16_t radius = py+iRadiusStart;
			double angle = px*2.0*M_PI/pWidth;
			uint16_t x = round(radius*cos(angle)+width/2-0.5);
			uint16_t y = round(radius*sin(angle)+height/2-0.5);
			
			if(x>=0 && x<width && y>=0 && y<height)
			{
				*(pointer++) = x;
				*(pointer++) = y;
			}
			else // set to midpoint, which should set to black
			{
				*(pointer++) = width/2;
				*(pointer++) = height/2;
			}
		}
	}
	
	/// Generate OuterTrafo
	pointer = outerPannoTrafo;
	for(uint16_t py=0; py<pHeightOuter; py++)
	{
		for(uint16_t px=0; px<pWidth; px++)
		{
			uint16_t radius = py+oRadiusEnd;
			double angle = px*2.0*M_PI/pWidth;
			uint16_t x = round(radius*cos(angle)+width/2-0.5);
			uint16_t y = round(radius*sin(angle)+height/2-0.5);
			
			if(x<0 || x>=width || y<0 || y>=height)
			{
				*(pointer++) = width/2;
				*(pointer++) = height/2;
			}
			else
			{
				*(pointer++) = x;
				*(pointer++) = y;
			}
		}
	}
	cout << "End FilterExtractPanno Constructor" << endl;
}


/**
 ** Extract a cartesien image out of a polar image
 **/
void FilterExtractPanno::process(unsigned char *&src, unsigned char *&inner, unsigned char *&outer, unsigned char *&panno)
{
	uint16_t x,y;
	
	// Made the center totaly black as reference for areas out of the image
	src[width/2 + height*width/2] = 0;
	
	// Convert inner image
	uint16_t * pointer	= innerPannoTrafo;
	for(uint16_t py=0; py<pHeightInner; py++)
	{
		for(uint16_t px=0; px<pWidth; px++)
		{
			x = *(pointer++);
			y = *(pointer++);
			
			innerPanno[px+py*pWidth]	= src[x+y*width];
			pannoImage[px+py*pWidth]	= src[x+y*width];
		}
	}
	
	// Convert outer image
	pointer = outerPannoTrafo;
	for(uint16_t py=0; py<pHeightOuter; py++)
	{
		for(uint16_t px=0; px<pWidth; px++)
		{
			x = *(pointer++);
			y = *(pointer++);
			
			outerPanno[px+py*pWidth] 	= src[x+y*width];
			pannoImage[px+py*pWidth + pHeightInner*pWidth]	= src[x+y*width];
		}
	}
	
	inner = innerPanno;
	outer = outerPanno;
	panno = pannoImage;
	
// 	pannoDisplay->displayFrameGRAY(pannoImage);
}




FilterExtractPanno::~FilterExtractPanno()
{
	printf("Destructor of FilterExtractImages\n");

	if(pannoDisplay != NULL)		delete pannoDisplay;
	if(pannoImage != NULL)		delete[] pannoImage;
	if(innerPanno != NULL)		delete[] innerPanno;
	if(innerPannoTrafo != NULL)	delete[] innerPannoTrafo;
	if(outerPanno != NULL)		delete[] outerPanno;
	if(outerPannoTrafo != NULL)	delete[] outerPannoTrafo;
}

