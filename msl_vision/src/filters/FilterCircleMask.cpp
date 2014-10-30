/*
 * $Id: FilterYUVToRGB.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "FilterGauss.h"

#include <algorithm>
#include <string>
#include <cmath>

FilterCircleMask::FilterCircleMask(int width, int height):Filter(OF_ZERO, width, height) {
	output = (unsigned int*) malloc(((width*height/32)*sizeof(int));
	memset(output,0,9600);
}

FilterCircleMask::~FilterCircleMask(){

	cleanup();

}
		

unsigned int* FilterCircleMask::process(unsigned int width, unsigned int height, unsigned int mx, unsigned int my, unsigned int r){
	
	for (int i = 0; i<width; i++) {
		for (int j = 0; j<height; j++) {
			if (sqrt(i-mx)*(i-mx)+(j-my)*(j-my)<radius) {
				*(output) = (*(output) & 1)<<1
			}
		}
	}
	
}



void FilterCircleMask::init(){


}

void FilterCircleMask::cleanup(){
	delete output;	
}

