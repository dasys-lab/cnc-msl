/*
 * $Id: FilterYUVExtractImages.h 2124 2007-04-14 17:01:41Z jewollen $
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
#ifndef FilterYUVExtractImages_H
#define FilterYUVExtractImages_H


#include "Filter.h"

class FilterYUVExtractImages  : public Filter {


	public:
		FilterYUVExtractImages(int width_, int height_, int refx_, int refy_, int areaWidth_, int areaHeight_);
		~FilterYUVExtractImages();
		
		void process(unsigned char * src, unsigned char* & gray_image_, unsigned char* & uv_image_);
		void process(unsigned char * src, unsigned char*  & gray_image_, unsigned char* & uv_image_, unsigned char* & roi_image_);
		 
	protected:

		void init();
		void initROI();
		void cleanup();

		int width;
		int height;

		int refx;
		int refy;

		int areaWidth;
		int areaHeight;

		unsigned char * uv_image;
		unsigned char * gray_image;
		unsigned char * roi_image;
		unsigned char * lookupTable;
		unsigned char * lookupTableROI;

		

};




#endif

