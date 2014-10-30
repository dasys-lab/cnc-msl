/*
 * $Id: FilterSobelGradient.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "FilterSoftHDR.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>

using namespace std;

FilterSoftHDR::FilterSoftHDR(int width, int height):Filter(OF_GRAY, width, height){
	result = (unsigned char*) malloc(width*height*2);
	cmpLU = (unsigned char*) malloc(256);
	
	for(int i=0; i<256; i++) {
		int value = fabs(i-127-0);
		if(i<127) value=(value*3)/2;
		value = value>255?255:value;
		cmpLU[i] = value;
		//if(i<50) cmpLU[i]=cmpLU[i] + 50;
		//if(i>180) cmpLU[i]=cmpLU[180];
	}
  
	init();
}



FilterSoftHDR::~FilterSoftHDR(){
	cleanup();
}
		
unsigned char * FilterSoftHDR::process(unsigned char * curr, unsigned char * prev, int width, int height) {

	unsigned char * tgt = prev;
	unsigned char * p = prev;
	unsigned char * c = curr;
	
	for(int i=0; i<width*height*2; i+=2) {
		//std::cout << i << " " << (int)*p << " " << (int)*c << " " << (int)*tgt << endl;
		if(cmpLU[*(c+1)] < cmpLU[*(p+1)]) {
			*tgt = *c;
			tgt++; c++;
			*tgt = *c;
			tgt++; c++;
			p++; p++;
		}
		else {
			*tgt = *p;
			tgt++; p++;
			*tgt = *p;
			tgt++; p++;
			c++; c++;
		}
	}
	

	return prev;
}



void FilterSoftHDR::init() {
}


void FilterSoftHDR::cleanup(){
	free(result);
}

