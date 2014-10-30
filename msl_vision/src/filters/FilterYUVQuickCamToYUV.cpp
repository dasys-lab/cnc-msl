/*
 * $Id: FilterYUVQuickCamToYUV.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "FilterYUVQuickCamToYUV.h"

#include <stdio.h>


FilterYUVQuickCamToYUV::FilterYUVQuickCamToYUV(int width, int height):Filter(OF_YUV422, width, height){

	init();

}



FilterYUVQuickCamToYUV::~FilterYUVQuickCamToYUV(){

	cleanup();

}
		

unsigned char * FilterYUVQuickCamToYUV::process(unsigned char * src, unsigned int imagesize){

	unsigned char * tgt = outputBuffer;

	unsigned char y1 = 0;
	unsigned char y2 = 0;
	unsigned char u = 0;
	unsigned char v = 0;

	for(unsigned int i = 0; i < imagesize/4; i++){
		
		y1 = *src++;
		u = *src++;
		y2 = *src++;
		v = *src++;

		*tgt++ = u;
		*tgt++ = y1;
		*tgt++ = v;
		*tgt++ = y2;

	}

	return outputBuffer;
}



void FilterYUVQuickCamToYUV::init(){


}


void FilterYUVQuickCamToYUV::cleanup(){


}

