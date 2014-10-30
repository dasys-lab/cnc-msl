/*
 * $Id: FilterSegToRGB.cpp 1870 2007-02-28 12:19:42Z rreichle $
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
#include "FilterSegToRGB.h"
#include "../helpers/Lookuptable.h"

#include <algorithm>

FilterSegToRGB::FilterSegToRGB(int width, int height):Filter(OF_RGB, width, height){

	init();

}



FilterSegToRGB::~FilterSegToRGB(){

	cleanup();

}
		

unsigned char * FilterSegToRGB::process(unsigned char * src, unsigned int imagesize){

	unsigned char * tgt = outputBuffer;	

	unsigned char color;

	for(unsigned int i = 0; i < imagesize; i++){
		color = *src++;
		if(color == COLOR_UDEF){
			*tgt++ = 255;
			*tgt++ = 255;
			*tgt++ = 255;
		}
		else if(color == COLOR_RED){
			*tgt++ = 255;
			*tgt++ = 0;
			*tgt++ = 0;
		}
		else if(color == COLOR_BLACK){
			*tgt++ = 0;
			*tgt++ = 0;
			*tgt++ = 0;
		}
		else if(color == COLOR_BLUE){
			*tgt++ = 0;
			*tgt++ = 0;
			*tgt++ = 255;
		}
		else if(color == COLOR_YELLOW){
			*tgt++ = 255;
			*tgt++ = 255;
			*tgt++ = 0;
		}
		else if(color == COLOR_GREEN){
			*tgt++ = 0;
			*tgt++ = 255;
			*tgt++ = 0;
		}

	}

	return outputBuffer;

}



void FilterSegToRGB::init(){


}


void FilterSegToRGB::cleanup(){


}

