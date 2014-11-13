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
#include "FilterHistoLin.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>

using namespace std;

FilterHistoLin::FilterHistoLin(int width, int height):Filter(OF_GRAY, width, height){

	AreaLookup = (unsigned char *) malloc(width*height*sizeof(unsigned char));

	FILE * areafile = fopen("ReduceAreaLookup.dat", "r");
	if(areafile != NULL){

		int n = fread(AreaLookup, sizeof(char), width*height, areafile);
		fclose(areafile);
		printf("AreaLookup: %d bytes read\n", n);
	}
	else{
		printf("Area Lookup File not found\n");

	}
	init();

}



FilterHistoLin::~FilterHistoLin(){
	cleanup();
}
		
unsigned char * FilterHistoLin::process(unsigned char * src, int width, int height, int threshold) {

	unsigned char * tgt = outputBuffer;
	static int histogram[256];
	for(int i = 0; i<256; i++) {
		histogram[i]=0;
	}

	for(int i = 1; i<(height*width); i++) {
		histogram[src[i]]++;
	}

	for(int i=1; i<256; i++) {
		histogram[i] = (histogram[i]+histogram[i-1]);
	}
	for(int i=1; i<256; i++) {
		histogram[i] = (histogram[i]*255)/((int)(width*height));
	}

	//cout << histogram[10] << "\t" << histogram[21] << "\t" << histogram[32] << "\t" << histogram[43] << "\t" << endl;

	for(int i = 1; i < (height*width); i++){
		tgt[i] = histogram[src[i]];
	}

	return outputBuffer;
}



void FilterHistoLin::init(){
}


void FilterHistoLin::cleanup(){
	if(AreaLookup != NULL)
		free(AreaLookup);
}

