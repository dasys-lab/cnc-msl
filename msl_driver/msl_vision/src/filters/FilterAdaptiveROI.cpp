/*
 * $Id: FilterAdaptiveROI.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "FilterAdaptiveROI.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

FilterAdaptiveROI::FilterAdaptiveROI(int width, int height, int area_):Filter(OF_ZERO, width, height){

	area = area_;
	range = area/10;

	histogram = (unsigned short *) malloc(256*sizeof(unsigned short));

	maxValues = (unsigned short *) malloc(range*range*sizeof(unsigned short));
	bzero(maxValues, range*range*sizeof(unsigned short));

	maxIndexX = (unsigned short *) malloc(range*range*sizeof(unsigned short));
	maxIndexY = (unsigned short *) malloc(range*range*sizeof(unsigned short));

	segImage = (unsigned char *) malloc(area*area);

	init();

}



FilterAdaptiveROI::~FilterAdaptiveROI(){

	cleanup();

}
		

unsigned char * FilterAdaptiveROI::process(unsigned short * src){


	unsigned short * tgt = src;
	unsigned short * ptr = src;

	bzero(maxValues, range*range*sizeof(unsigned short));

	for(unsigned short i = 0; i < area; i++){

		short indexI = i/10;

		for(unsigned short j = 0; j < area; j++){
			
			
			short indexJ = j/10;

			if(*ptr > maxValues[indexI*range + indexJ]){

				maxIndexX[indexI*range + indexJ] = i;
				maxIndexY[indexI*range + indexJ] = j;
				maxValues[indexI*range + indexJ] = *ptr;
			}

			ptr++;
		}

	}

	bzero(histogram, 256*sizeof(unsigned short));

	for(int i = 0; i < range*range; i++){		
		histogram[maxValues[i]]++;
	}

	double percentile = range*range*0.95;
	printf("Percentile: %f\n", percentile);

	int sum = 0;
	int index = 0;

	for(int i = 0; i < 256; i++){
		sum += histogram[i];
		if(sum >= percentile)
			break;
		index++;

	}

	//index = 170;

	printf("Percentile value = %d\n", index);

	int frame = 15;

	int checkSize = 2;

	bzero(segImage, area*area);


	for(int i = 2; i < range-2; i++){
		for(int j = 2; j < range - 2; j++){
			
			if(maxValues[i*range + j] >= index && maxValues[i*range + j] >= 1){

				int checkCounter = 0;

				for(unsigned int a = maxIndexX[i*range + j] - checkSize; a < maxIndexX[i*range + j] + checkSize; a++){
					for(unsigned int b = maxIndexY[i*range + j] - checkSize; b < maxIndexY[i*range + j] + checkSize; b++){
						if(src[a*area + b] > 0.90*maxValues[i*range + j])
							checkCounter++;
					}
				}

				if(checkCounter > 6){

					for(unsigned int a = maxIndexX[i*range + j] - frame; a < maxIndexX[i*range + j] + frame; a++){
						for(unsigned int b = maxIndexY[i*range + j] - frame; b < maxIndexY[i*range + j] + frame; b++){
							if(src[a*area + b] > 0.85*maxValues[i*range + j])
								segImage[a*area + b] = 255;
						}
					}

				}
			}
		
		}
	}


	return segImage;

}



void FilterAdaptiveROI::init(){


}


void FilterAdaptiveROI::cleanup(){

	free(histogram);
	free(maxIndexX);
	free(maxIndexY);
	free(maxValues);
	free(segImage);
}

unsigned char * FilterAdaptiveROI::getResult(){

	return segImage;


}


