/*
 * $Id: FilterYUVExtractSubImages.h 2124 2007-04-14 17:01:41Z jewollen $
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
#ifndef FilterYUVExtractSubImages_H
#define FilterYUVExtractSubImages_H

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>


#include "Filter.h"
#include <ros/console.h>
#include <SystemConfig.h>

using namespace std;

class FilterYUVExtractSubImages  : public Filter {


	public:
		FilterYUVExtractSubImages(int width, int height, int area_);
		~FilterYUVExtractSubImages();
        void loadAllLookupTables();
		void loadLookupTable(string filename, unsigned char* lookup);
		void loadLookupTable(string filename, unsigned char* lookup, int &min, int &max);
		
		void process(unsigned char * src, unsigned int width, unsigned int height, unsigned int mx, unsigned int my, unsigned char* & gray_image_, unsigned char* & uv_image_, unsigned char* & roi_image_, unsigned char* & roi_image_Roland_, unsigned char* &shadowlessGrey_);
        void process(unsigned char * src, unsigned int width, unsigned int height, unsigned int mx, unsigned int my, unsigned char*  & gray_image_, unsigned char* & uv_image_, unsigned char* & roi_image_);

        void setLookupTableValue(int index, int value);
        void setLookupTableROIValue(int index, int value);
        void setLookupTableROIRolandValue(int index, int value);
        void setLookupTableUVYValue(int index, int value);

        int getLookupTableValue(int index);
        int getLookupTableROIValue(int index);
        int getLookupTableROIRolandValue(int index);
        int getLookupTableUVYValue(int index);
	protected:

		void init();
		void initROI();
		void initRoland();
		void cleanup();

		int UVYMIN, UVYMAX;
		int area;
		unsigned char * uv_image;
		unsigned char * gray_image;
		unsigned char * roi_image;
		unsigned char * roi_image_Roland;
		unsigned char * shadowlessGrey;
		unsigned char * lookupTable;
		unsigned char * lookupTableROI;
		unsigned char * lookupTableROIRoland;
		unsigned char * lookupTableUVY;
        unsigned char * lookupTableGreen;

};




#endif

