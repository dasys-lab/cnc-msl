/*
 * $Id: FilterGrayToDarkSeg.cpp 2142 2007-04-15 10:49:00Z jewollen $
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
#include "FilterGrayToDarkSeg.h"

#include <stdio.h>
#include <string>
#include <iostream>
#include <math.h>
#include "../helpers/Lookuptable.h"




FilterGrayToDarkSeg::FilterGrayToDarkSeg(int width, int height):Filter(OF_GRAY, width, height){
	
	init();
	
	

}



FilterGrayToDarkSeg::~FilterGrayToDarkSeg(){

	cleanup();

}
		

unsigned char * FilterGrayToDarkSeg::process(unsigned char * src, unsigned char * src_uv, unsigned int width, unsigned int height, ImageMaskHelper & maskHelper){

	unsigned char * tgt = outputBuffer;

	unsigned char * mask = maskHelper.getLookupTable();

	unsigned int mx = height/2;
	unsigned int my = width/2;

	unsigned int frame = 25;

	unsigned int sum = 0;	

	for(unsigned int i = mx - frame; i < mx + frame; i++){
		for(unsigned int j = my - frame; j < my + frame; j++){
						
			sum += src[i*width + j];
		
		}
	}
			
	double threshold = ((double)sum)/((double)(4*frame*frame));

	double addThreshold = threshold - 10.0;
	if(addThreshold < 3.0)
		addThreshold = 3.0;
	if(addThreshold > 10.0)
		addThreshold = 10.0;

	//threshold += addThreshold;

	printf("FilterGrayToSeg: threshold = %f\n", threshold);

	unsigned char threshold_ind = (unsigned char) lrint(threshold);
	
	for(unsigned int i = 0; i < height*width; i++){


		if(*src < threshold_ind && /*mask > 0 &&*/ *src_uv < 138)
			*tgt++ = COLOR_BLACK;
		else
			*tgt++ = COLOR_UDEF;

		src++;
		src_uv++;
		mask++;
	}

	return outputBuffer;

}



void FilterGrayToDarkSeg::init(){


}


void FilterGrayToDarkSeg::cleanup(){

}

