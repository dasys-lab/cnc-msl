/*
 * $Id: FilterAdaptiveROIDirected.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "FilterAdaptiveROIDirected.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

FilterAdaptiveROIDirected::FilterAdaptiveROIDirected(int width, int height, int areaWidth_, int areaHeight_):Filter(OF_ZERO, width, height){

	areaWidth = areaWidth_;
	areaHeight = areaHeight_;
	rangeWidth = areaWidth/10;
	rangeHeight = areaHeight/10;

	histogram = (unsigned short *) malloc(256*sizeof(unsigned short));

	maxValues = (unsigned short *) malloc(rangeWidth*rangeHeight*sizeof(unsigned short));
	bzero(maxValues, rangeWidth*rangeHeight*sizeof(unsigned short));

	maxIndexX = (unsigned short *) malloc(rangeWidth*rangeHeight*sizeof(unsigned short));
	maxIndexY = (unsigned short *) malloc(rangeWidth*rangeHeight*sizeof(unsigned short));

	segImage = (unsigned char *) malloc(areaWidth*areaHeight);

	init();

}



FilterAdaptiveROIDirected::~FilterAdaptiveROIDirected(){

	cleanup();

}
		

unsigned char * FilterAdaptiveROIDirected::process(unsigned short * src){


	unsigned short * tgt = src;
	unsigned short * ptr = src;

	bzero(maxValues, rangeWidth*rangeHeight*sizeof(unsigned short));

	for(unsigned short i = 0; i < areaHeight; i++){

		short indexI = i/10;

		for(unsigned short j = 0; j < areaWidth; j++){
			
			
			short indexJ = j/10;

			if(*ptr > maxValues[indexI*rangeWidth + indexJ]){

				maxIndexX[indexI*rangeWidth + indexJ] = i;
				maxIndexY[indexI*rangeWidth + indexJ] = j;
				maxValues[indexI*rangeWidth + indexJ] = *ptr;
			}

			ptr++;
		}

	}

	bzero(histogram, 256*sizeof(unsigned short));

	for(int i = 0; i < rangeWidth*rangeHeight; i++){		
		histogram[maxValues[i]]++;
	}

	double percentile = rangeWidth*rangeHeight*0.95;
	printf("Percentile: %f\n", percentile);

	int sum = 0;
	int index = 0;

	for(int i = 0; i < 256; i++){
		sum += histogram[i];
		if(sum >= percentile)
			break;
		index++;

	}


	if(index < 180)
		index = 180;

	//index = 170;

	printf("Percentile Adaptive value = %d\n", index);

	int frame = 15;

	int checkSize = 2;

	bzero(segImage, areaWidth*areaHeight);


	for(int i = 2; i < rangeHeight-2; i++){
		for(int j = 2; j < rangeWidth - 2; j++){
			
			if(maxValues[i*rangeWidth + j] >= index){

				int checkCounter = 0;

				for(unsigned int a = maxIndexX[i*rangeWidth + j] - checkSize; a < maxIndexX[i*rangeWidth + j] + checkSize; a++){
					for(unsigned int b = maxIndexY[i*rangeWidth + j] - checkSize; b < maxIndexY[i*rangeWidth + j] + checkSize; b++){
						if(src[a*areaWidth + b] > 0.90*maxValues[i*rangeWidth + j])
							checkCounter++;
					}
				}

				if(checkCounter > 6){

					for(unsigned int a = maxIndexX[i*rangeWidth + j] - frame; a < maxIndexX[i*rangeWidth + j] + frame; a++){
						for(unsigned int b = maxIndexY[i*rangeWidth + j] - frame; b < maxIndexY[i*rangeWidth + j] + frame; b++){
							if(src[a*areaWidth + b] > 0.75*maxValues[i*rangeWidth + j])
								segImage[a*areaWidth + b] = 255;
						}
					}

				}
			}
		
		}
	}


	return segImage;

}



void FilterAdaptiveROIDirected::init(){


}


void FilterAdaptiveROIDirected::cleanup(){

	free(histogram);
	free(maxIndexX);
	free(maxIndexY);
	free(maxValues);
	free(segImage);
}

unsigned char * FilterAdaptiveROIDirected::getResult(){

	return segImage;


}


