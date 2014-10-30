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
#include "FilterSobelDir.h"


FilterSobelDir::FilterSobelDir(int width, int height):Filter(OF_GRAY, width, height){

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

	box = (unsigned char *) malloc(width*height*sizeof(unsigned char));
	xbox = (int *) malloc(width*height*sizeof(int));
	ybox = (int *) malloc(width*height*sizeof(int));
}


// For 3D
FilterSobelDir::FilterSobelDir(ImageSize size):Filter(OF_GRAY, size.width, size.height)
{
	width = size.width;
	height = size.height;
	
	AreaLookup = new unsigned char [width*height];

	FILE * areafile = fopen("ReduceAreaLookup.dat", "r");
	if(areafile != NULL)
	{
		int n = fread(AreaLookup, sizeof(char), width*height, areafile);
		fclose(areafile);
		printf("AreaLookup: %d bytes read\n", n);
	}
	else
	{
		printf("Area Lookup File not found\n");
	}
	
	xbox	= new int [width*height];
	ybox	= new int [width*height];
	
	buffer	= new unsigned char [width*height];
}


FilterSobelDir::~FilterSobelDir(){
	cleanup();
}
		
int inline FilterSobelDir::sign(int s) {
        return (s<0) ? -1 : 1;
}

int inline FilterSobelDir::sign2(int s) {
	return (s<0) ? -2 : 2;
}

int inline FilterSobelDir::dir(int gx, int gy, int threshold) {
	if(gx*gx + gy*gy > threshold) {
	//if(gx > threshold || gy > threshold) {
	//if(gx + gy > threshold) {
		int grad;
		//gx = gx*sign(gx);
		//gy = gy*sign(gy);
		//if(gx == 0)
		//	grad = gy;
		if(gy == 0)
			return 121+120*sign(gy);
		else {
			grad = 121+((60*gx)/gy);
		}
		if (grad > 240) return 240;
		else if(grad < 1) return 1;
		return grad;
	}
	else {
		return 0;
	}
}

// For 3D
// gx,gy [-255..255]
int8_t inline FilterSobelDir::dir(int8_t gx, int8_t gy, uint8_t threshold)
{
	if(gx*gx + gy*gy > threshold)
	{
		int grad;
		
		if(gx == 0)
		{
			return 255;
		}
		else
		{
			// Normalizing between 1 .. 255
			// with expexted values between -4 and 4
			grad = 16.0 * gy / gx;
			
			if (grad>128)
				grad = 128;
			if (grad<-128)
				grad = -128;
			
			grad += 127;
			
			if (grad <= 1)
				return 1;
			else
				return grad;
		}
	}
	else
	{
		return 0;
	}
}

unsigned char * FilterSobelDir::process(unsigned char * src, unsigned char * mask, std::vector<ROIData> &roiData, int width, int height, int threshold, unsigned char minColor) {

	unsigned char * tgt = outputBuffer;
	int gx, gy;
	int maxIndex = (width*height)-(1+width);
	ROIData dat;
	int index;
	int sum=0;

	memset(tgt, 0, width*height);
	memset(box, 0, width*height);
	

	for(int i=0; i<roiData.size(); i++){
		dat = roiData[i];
		if(dat.left>4 && dat.top>4 && dat.bottom<height-4 && dat.right<width-4) { 
			for(int x=dat.left-2; x<dat.right+2; x++) {
				for(int y=dat.top-2; y<dat.bottom+2; y++) {
					index = x + y*width;
					sum=0;
					for(int l=-2; l<=2; l++) {
						sum += src[index+l];
					}
					//sum += src[index];
					box[index] = sum/5;
				}
			}
	
			for(int x=dat.left; x<dat.right; x++) {
				for(int y=dat.top; y<dat.bottom; y++) {
					index = x + y*width;
					sum=0;
					for(int l=-2; l<=2; l++) {
						sum += box[index+l*width];
					}
					//sum += box[index];
					box[index] = sum/5;
				}
			}
		}
	}

	for(int i=0; i<roiData.size(); i++){
		dat = roiData[i];
		for(int x=dat.left; x<dat.right; x++) {
			for(int y=dat.top; y<dat.bottom; y++) {
				index = x + y*width;
				//if(mask[index] < minColor) {
				if(mask[index] < minColor && mask[index+1] < minColor && mask[index-1] < minColor &&  mask[index+width] < minColor && mask[index-width] < minColor) {
					tgt[index] = 0;
					continue;
				}
				gx = box[index + 1] - box[index - 1];
				gy = box[index + width] - box[index-width];
		
				tgt[index] = dir(gx, gy, threshold);
				//tgt[index] = box[index];
			}
		}
	}			

/*	for(int index = width+1; index < maxIndex; index++){
		if(mask[index] < minColor) {
			tgt[index] = 0;
			continue;
		}
		gx = src[index + 1] - src[index - 1];
		gy = src[index + width] - src[index-width];

		tgt[index] = dir(gx, gy, threshold);
	}			
*/
	return outputBuffer;
}

/// For 3D
void FilterSobelDir::process(unsigned char * src, unsigned char * &dst, uint8_t threshold, int16_t size)
{
	int8_t gx, gy;
	uint32_t index;
	uint32_t sum=0;

	memset(buffer, 0, width*height);
	memset(xbox, 0, width*height*sizeof(int));
	memset(ybox, 0, width*height*sizeof(int));
	
	for(uint16_t x=size; x<width-size; x++)
	{
		for(uint16_t y=0; y<height; y++)
		{
			index = x + y*width;
			sum=0;
			for(int16_t l=-size; l<=size; l++)
			{
				sum += src[index+l];
			}
			xbox[index] = sum;
		}
	}
	
	for(uint16_t x=size; x<width-size; x++)
	{
		for(uint16_t y=size; y<height-size; y++)
		{
			index = x + y*width;
			sum=0;
			for(int16_t l=-size; l<=size; l++)
			{
				sum += xbox[index+l*width];
			}
			ybox[index] = sum;
		}
	}

	int sizeSqr = (2*size+1)*(2*size+1);
	for(uint16_t x=size; x<width-size; x++)
	{
		for(uint16_t y=size; y<height-size; y++)
		{
			index = x + y*width;
			
			gx = (ybox[index + 1] - ybox[index - 1]) / sizeSqr / 2;
			gy = (ybox[index + width] - ybox[index-width]) / sizeSqr / 2;
	
			buffer[index] = dir(gx, gy, threshold);
		}
	}
	dst = buffer;
}


void FilterSobelDir::cleanup(){
	if(AreaLookup != NULL)	delete(AreaLookup);
	if(buffer != NULL)	delete(buffer);
	if(box != NULL)	delete(box);
	if(xbox != NULL)	delete(xbox);
	if(ybox != NULL)	delete(ybox);
}

