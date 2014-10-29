/*
 * $Id: FilterExtractLineImage.cpp 2142 2007-04-15 10:49:00Z jewollen $
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
#include "FilterExtractLineImage.hpp"

#include <stdio.h>
#include <iostream>
#include <SystemConfig.h>

using std::cout;
using std::endl;

extern bool online;
extern bool color;

FilterExtractLineImage::FilterExtractLineImage(void)
{
	cout << "Start FilterExtractLineImage Constructor" << endl;
	
	castor::SystemConfigPtr sc = castor::SystemConfig::getInstance();
	castor::Configuration *vision3D = (*sc)["Vision3D"];
	
	cout << "Image" << endl;
	cout << "\tWidth_";
	width	= vision3D->get<uint16_t>("Image", "Width", NULL);
	cout << "\tHeight_";
	height	= vision3D->get<uint16_t>("Image", "Height", NULL);
	
	init();
	
	cout << "End FilterExtractLineImage Constructor" << endl;
}


FilterExtractLineImage::FilterExtractLineImage(struct ImageSize size)
{
	cout << "FilterExtractLineImage Constructor with ImageSize: " << size.width << " times " << size.height << endl;
	
	width	= size.width;
	height	= size.height;
	
	init();
	
	cout << "End FilterExtractLineImage Constructor" << endl;
}



void FilterExtractLineImage::init()
{
	/// Memory Alloc
	lineImage	= new unsigned char [width*height];
	
	/// Setup LineLookupTable
	lineLookupTable	= new unsigned char [256*256*256];
	if(lineLookupTable == NULL)	printf("lineLookupTable = null!!!\n");
	
	int16_t value;
	for(int y=0; y<256; y++)
	{
		for(int u=0; u<256; u++)
		{
			for( int v=0; v<256; v++)
			{
				if(online)
				{
					value = y*2 - abs(u-128) - abs(v-128);	// White
					if( value > 280 )
						lineLookupTable[y*256*256 + u*256 + v] = 255;
					else
						lineLookupTable[y*256*256 + u*256 + v] = 0;
				}
				else
				{
					value = y*2 - abs(u-128) - abs(v-128);	// White
					if( value > 250 )
						lineLookupTable[y*256*256 + u*256 + v] = 255;
					else
						lineLookupTable[y*256*256 + u*256 + v] = 0;
				}
			}
		}
	}
}



FilterExtractLineImage::~FilterExtractLineImage()
{
	cout << "Destructor of FilterExtractLineImage" << endl;
	
	if( lineImage != NULL )		delete[] lineImage;
	if( lineLookupTable != NULL )	delete[] lineLookupTable;
}

void FilterExtractLineImage::setLineLookupTableValue(int index, int value) {
	lineLookupTable[index] = value;
}

int FilterExtractLineImage::getLineLookupTableValue(int index) {
	return lineLookupTable[index];
}


void FilterExtractLineImage::process(unsigned char * &src, unsigned char * &lineImage_)
{
	unsigned char * linePointer	= lineImage;
	unsigned char * ptr	= src;
	
	register unsigned char u = 0;
	register unsigned char y = 0;
	register unsigned char v = 0;
	
	if(color)
	{
		ptr += 2*width;
		for(uint16_t i = 0+1; i<height-1; i++)
		{
			ptr++;
			ptr++;
			
			for(uint16_t j = 0+1; j<width-1; j++)
			{
				// Get uv
				if(j%2==0)
					u = *ptr++;
				else
					v = *ptr++;
				
				// Get gray
				y = *ptr++;
				
				// Store mask
				linePointer[j+i*width] = lineLookupTable[y*256*256 + u*256 + v];
			}
			ptr++;
			ptr++;
		}
	}
	else
	{
		for(uint32_t i=0; i<height*width; i++)
		{
			if(src[i]>100)
				linePointer[i] = 255;
			else
				linePointer[i] = 0;
		}
	}
	
	lineImage_	= lineImage;
}
