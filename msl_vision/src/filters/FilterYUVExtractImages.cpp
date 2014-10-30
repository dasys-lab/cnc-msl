/*
 * $Id: FilterYUVExtractImages.cpp 2142 2007-04-15 10:49:00Z jewollen $
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
#include "FilterYUVExtractImages.h"

#include <stdio.h>
#include <string>
#include <iostream>
#include <math.h>

//#include "floatfann.h"



FilterYUVExtractImages::FilterYUVExtractImages(int width_, int height_, int refx_, int refy_, int areaWidth_, int areaHeight_):Filter(OF_ZERO, width_, height_){

	printf("before malloc!!!\n");

	width = width_;
	height = height_;
	
	refx = refx_;
	refy = refy_;

	areaWidth = areaWidth_;
	areaHeight = areaHeight_;

	gray_image = (unsigned char *) malloc(areaHeight*areaWidth);
	uv_image = (unsigned char *) malloc(areaHeight*areaWidth);
	roi_image = (unsigned char *) malloc(areaHeight*areaWidth);

	if(gray_image == NULL)
		printf("gray_image = null!!!\n");

	if(uv_image == NULL)
		printf("uv_image = null!!!\n");

	lookupTable = (unsigned char *) malloc(256*256);
	lookupTableROI = (unsigned char *) malloc(256*256);

	if(lookupTable == NULL)
		printf("lookupTable = null!!!\n");
	
	if(lookupTableROI == NULL)
		printf("lookupTable = null!!!\n");

	init();
	initROI();

}



FilterYUVExtractImages::~FilterYUVExtractImages()
{
	cleanup();
}
		

void FilterYUVExtractImages::process(unsigned char * src, unsigned char*  & gray_image_, unsigned char* & uv_image_){
		

		printf("FilterYUVExtractImages - refx=%d refy=%d areaWidth=%d areaHeight=%d\n", refx, refy, areaWidth, areaHeight);

		unsigned char * gray_tgt = gray_image;
		unsigned char * uv_tgt = uv_image;

		int startIndexX = refx;
		int startIndexY = refy;

		//XXX nicht schön, aber macht die Sache einfacher!

		if(startIndexY % 2 != 0)
			startIndexY++;

		register unsigned char u = 0;
		register unsigned char y = 0;
		register unsigned char v = 0;

		register int color;

		for(int i = 0; i < areaHeight; i++){

			unsigned char * ptr = &(src[((startIndexX + i)*width + startIndexY)*2]);
			for(int j = 0; j < areaWidth; j++){
				
				if(j%2==0)	
					u = *ptr++;
				else
					v = *ptr++;
				y = *ptr++;
				color = u*256 + v;
				
//				printf("i = %d j = %d\n", i, j);

				if(y < 180)
					*uv_tgt++ = lookupTable[color];
				else
					*uv_tgt++ = 0;
				*gray_tgt++ = y;

/*

				if(*ptr < 180)
					*uv_tgt++ = lookupTable[u*256 + v];
				else
					*uv_tgt++ = 0;
				*gray_tgt++ = *ptr++;
*/
			}
		}

		gray_image_ = gray_image;
		uv_image_ = uv_image;
}



void FilterYUVExtractImages::process(unsigned char * src, unsigned char*  & gray_image_, unsigned char* & uv_image_, unsigned char* & roi_image_){
		

		printf("FilterYUVExtractImages - refx=%d refy=%d areaWidth=%d areaHeight=%d\n", refx, refy, areaWidth, areaHeight);

		unsigned char * gray_tgt = gray_image;
		unsigned char * uv_tgt = uv_image;
		unsigned char * roi_tgt = roi_image;


		int startIndexX = refx;
		int startIndexY = refy;

		//XXX nicht schön, aber macht die Sache einfacher!

		if(startIndexY % 2 != 0)
			startIndexY++;

		register unsigned char u = 0;
		register unsigned char y = 0;
		register unsigned char v = 0;

		register int color;
		for(int i = 0; i < areaHeight; i++){

			unsigned char * ptr = &(src[((startIndexX + i)*width + startIndexY)*2]);
			for(int j = 0; j < areaWidth; j++){
				
				if(j%2==0)	
					u = *ptr++;
				else
					v = *ptr++;
				y = *ptr++;
				color = u*256 + v;
				
//				printf("i = %d j = %d\n", i, j);

				if(y <= 255)
					*uv_tgt++ = lookupTable[color];
				else
					*uv_tgt++ = 0;
				*gray_tgt++ = y;


/*
				if(*ptr <= 255)
					*uv_tgt++ = lookupTable[color];
				else
					*uv_tgt++ = 0;
				*gray_tgt++ = *ptr++;
*/




				if(y <= 255)
					*roi_tgt++ = lookupTableROI[color];
				else
					*roi_tgt++ = 0;


/*

				if(*ptr <= 255)
					*roi_tgt++ = lookupTableROI[color];
				else
					*roi_tgt++ = 0;
*/




			}
		}

		gray_image_ = gray_image;
		uv_image_ = uv_image;
		roi_image_ = roi_image;
}



void FilterYUVExtractImages::init(){
	int center = 128;
	//220 - 50
	double angle = atan2(220 - center, 50 - center); 

// 	double minO=10;
// 	double maxO=-10;
// 
// 	fann_type input[2];
// 	fann_type *calc_out;
// 
// 	struct fann *ann = fann_create_from_file("fann/COI.net");
// 	for(int u = 0; u < 256; u++){
// 		for(int v = 0; v < 256; v ++){
// 			input[0] = v;
//     			input[1] = u;
// 
// 			calc_out = fann_run(ann, input);
// 
// 			double val = calc_out[0];
// 			if(val>maxO) maxO = val;
// 			if(val<minO) minO = val;
// 		}
// 	}

	for(int u = 0; u < 256; u++){
		for(int v = 0; v < 256; v ++){  


			/*int value = (int) lrint(cos(angle)*(u-center) + sin(angle)*(v-center)) + 128;
			double diffAngle = fabs(atan2(v-center, u-center) - angle);
			double distance = sqrt((v-center)*(v-center) + (u-center)*(u-center))/128.0;
			if(diffAngle > M_PI)
				diffAngle = fabs(diffAngle - 2.0*M_PI);
			value -= (int) lrint(diffAngle*180.0/M_PI*(0.5*distance + 0.5));

			//int value = v; //(int) lrint(sqrt(2.0)*((v - 128) - (u - 128)) + 128);
			//value = v;
			//value = (int) lrint((1.0/sqrt(2.0))*((v - 128) - (u - 128)) + 128);
			if(value < 0)
				value = 0;
			if(value > 255)
				value = 255;

			lookupTable[u*256 + v] = (unsigned char) value; */

			int value = (int) lrint(((cos(angle)*(u-center) + sin(angle)*(v-center)) * 128)*2.0);
			double diffAngle = fabs(atan2(v-center, u-center) - angle);
			double distance = sqrt((v-220)*(v-220) + (u-50)*(u-50));
			if(diffAngle > M_PI)
				diffAngle = fabs(diffAngle - 2.0*M_PI);
			value = (int) 255.0 - lrint(5.0*diffAngle*180.0/M_PI*(0.5*distance + 0.5));
			value = lrint(distance);


			//
                        int value2 = (int)lrint((pow(v,1.95) - u)/64.0);
                        value2 -= (int) lrint(0.7*diffAngle*180.0/M_PI);

			//value = ((value2/2) + (value/2));
			value = value2;
			//int value = v; //(int) lrint(sqrt(2.0)*((v - 128) - (u - 128)) + 128);
			//value = (int) lrint( (((double)(pow(v, 1.8)-u))/64.0)-0.1*diffAngle*(180.0/M_PI) );

			//value = (int) lrint( (((double)(pow(v, 1.80)-u))/64.0)-0.15*diffAngle*(180.0/M_PI) );

			//value = (int) lrint((1.0/sqrt(2.0))*((v - 128) - (u - 128)) + 128);

			//value = (int) lrint(((v - u) + 255.0)/2.0);

			//if(u > 110)
			//	value = 0;

			/* NN
			input[0] = v;
    			input[1] = u;

			calc_out = fann_run(ann, input);

			value = (int) ((minO+calc_out[0])*(256.0/(maxO-minO)));
			*/

			//value = 2*(v-u);
			if(value < 0)
				value = 0;
			if(value > 255)
				value = 255;

			lookupTable[u*256 + v] = (unsigned char) value; 
		}
	}


}

void FilterYUVExtractImages::initROI(){


	int center = 128;
	double angle = atan2(220 - center, 50 - center); 

	for(int u = 0; u < 256; u++){
		for(int v = 0; v < 256; v ++){  



			int value = (int) lrint(((cos(angle)*(u-center) + sin(angle)*(v-center)) * 128)*2.0);
			double diffAngle = fabs(atan2(v-center, u-center) - angle);
			double distance = sqrt((fabs(220-v))*(fabs(220-v)) + (fabs(50-u))*(fabs(50-u)));
			if(diffAngle > M_PI)
				diffAngle = fabs(diffAngle - 2.0*M_PI);
			value = (int) 255.0 - lrint(5.0*diffAngle*180.0/M_PI*(0.5*distance + 0.5));
			value = 255-lrint(3*distance);


			//
                        int value2 = (int)lrint((pow(v,1.75) - u)/64.0);
                        value2 -= (int) lrint(0.5*diffAngle*180.0/M_PI);

			value = ((value2/2) + (value/2))/1.5;
			//value = value2;
			if(value < 0)
				value = 0;
			if(value > 255)
				value = 255;

			lookupTableROI[u*256 + v] = (unsigned char) value; 





		}
	}


}



void FilterYUVExtractImages::cleanup(){
	if(gray_image != NULL)	free(gray_image);
	if(uv_image != NULL)	free(uv_image);
	if(roi_image != NULL)	free(roi_image);
	if(lookupTable != NULL)	free(lookupTable);
	if(lookupTableROI != NULL)	free(lookupTableROI);

}

