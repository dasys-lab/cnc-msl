/*
 * $Id: FilterSobelGradient.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef FilterSobelDir_H
#define FilterSobelDir_H

#include "../global/Types.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <stdint.h>

#include "Filter.h"

class FilterSobelDir  : public Filter {


	public:
		FilterSobelDir(int width, int height);
		FilterSobelDir(ImageSize size);
		~FilterSobelDir();
		
		unsigned char * process(unsigned char * src, unsigned char * mask, std::vector<ROIData> &roiData, int width, int height, int threshold, unsigned char minColor);
		void process(unsigned char* src, unsigned char*& dst, uint8_t threshold, int16_t size);

	protected:

		int inline sign(int s); 
		int inline sign2(int s);
		int inline dir(int gx, int gy, int threshold);
		int8_t inline dir(int8_t gx, int8_t gy, uint8_t threshold);

		void cleanup();
		
		uint16_t width;
		uint16_t height;
		
		unsigned char * box;
		int *xbox, *ybox;
		unsigned char * AreaLookup;
		
		unsigned char * buffer;

};




#endif

