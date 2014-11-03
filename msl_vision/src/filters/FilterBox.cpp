/*
 * $Id: FilterBox.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "FilterBox.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

FilterBox::FilterBox(int width, int height):Filter(OF_ZERO, width, height){

	boxImage = (unsigned short *) malloc(width*height*sizeof(unsigned short));
	bzero((unsigned char *) boxImage, width*height*sizeof(unsigned short));

	boxImageChar = (unsigned char *) malloc(width*height);
	bzero((unsigned char *) boxImageChar, width*height);

	imWidth = width;
	imHeight = height;

	init();

}



FilterBox::~FilterBox(){

	cleanup();

}
		

unsigned short * FilterBox::process(unsigned char * src, int width, int height){


	unsigned short * tgt = boxImage;
	bzero((unsigned char *) boxImage, width*height*sizeof(unsigned short));

	int counterPos = 0;
	int counterNeg = 0;

	for(int i = 1; i < height - 1; i++){

		tgt =  boxImage + i*width + 1;

		for(int j = 1; j < width-1; j++){

			if(src[i*width + j] >= 128){
	
				unsigned char * curr = src + i*width + j - 1;
				unsigned char * above = curr - width;
				unsigned char * below = curr + width;
	
				*tgt += *curr++;
				*tgt += *curr++;
				*tgt += *curr;
	
				*tgt += *above++;
				*tgt += *above++;
				*tgt += *above;
	
				*tgt += *below++;
				*tgt += *below++;
				*tgt += *below;
	
				*tgt++ /= 9;
				//counterPos++;
			}
			else {
				*tgt++ = src[i*width + j];
				//counterNeg++;
			}

		}
	}

	printf("counterPos = %d\n", counterPos);
	printf("counterNeg = %d\n", counterNeg);

	return boxImage;


}



void FilterBox::init(){


}


void FilterBox::cleanup(){

	free(boxImage);
	free(boxImageChar);
}


unsigned short * FilterBox::getResult(){

	return boxImage;
}

unsigned char * FilterBox::getResultChar(){

	unsigned char * tgt = boxImageChar;
	unsigned short * src = boxImage;

	for(int i = 0; i < imWidth*imHeight; i++){
		if(*src >= 0)
			*tgt++ = (unsigned char) *src++;
		else{
			*tgt++ = 0;
			src++;
		}

	}



	return boxImageChar;
}

