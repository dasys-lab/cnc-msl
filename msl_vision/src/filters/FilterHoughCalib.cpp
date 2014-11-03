/*
 * $Id: FilterHoughCalib.cpp 1987 2007-04-09 16:58:10Z rreichle $
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
#include "FilterHoughCalib.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define MX 240
#define MY 320

#define OX 150
#define OY 150
#define OR 240

#define DX 180
#define DY 340
#define DR 60


FilterHoughCalib::FilterHoughCalib(int width, int height):Filter(OF_GRAY, width, height){

	init();

}



FilterHoughCalib::~FilterHoughCalib(){

	cleanup();

}
		

unsigned char * FilterHoughCalib::process(unsigned char * src, int width, int height){

	unsigned char * tgt = outputBuffer;
	memcpy(tgt, src, width*height);

	int * HoughSpace = (int *) malloc(DX*DY*DR*sizeof(int));
	bzero(HoughSpace, DX*DY*DR*sizeof(int));

	for(int x = 0; x < height; x++){

		printf("*");

		for(int y = 0; y < width; y++){

			if(src[x*width + y] > 0 && ((x-MX)*(x-MX) + (y-MY)*(y-MY) > 180*180)){

				for(int r = 0; r < DR; r++){

					DrawCircleInc(x - OX, y - OY, r + OR, (int*) &(HoughSpace[r*DX*DY]), DY, DX);


				}

			}

		}

	}

	printf("\n");

	int maxValue = 0;
	int indX = 0;
	int indY = 0;
	int indR = 0;

	for(int r = 0; r < DR; r++){
		for(int x = 0; x < DX; x++){
			for(int y = 0; y < DY; y++){
				if(HoughSpace[r*(DX*DY) + x*(DY) + y] > maxValue){
					maxValue = HoughSpace[r*(DX*DY) + x*(DY) + y];
					indX = x;
					indY = y;
					indR = r;
				}

			}
		}
	}

	mx = indX + OX;
	my = indY + OY;
	radius = indR + OR;

	DrawCircle(mx, my, radius, tgt, width, height);
	
	printf("Hough Transformation Results MX: %d MY: %d Radius: %d\n\n", mx, my, radius);

	free(HoughSpace);

	return outputBuffer;


}



void FilterHoughCalib::init(){


}


void FilterHoughCalib::cleanup(){


}

void FilterHoughCalib::DrawCircleInc(int midX, int midY, int rad, int * space, int width, int height){



	int d = 3 - (2 * rad);
	int x = 0;
	int y = rad;

	while(y > x){

		if(midX + x >= 0 && midX + x < height && midY + y >= 0 && MY + y < width)
			space[(midX + x) * width + midY + y]++;
	
		if(midX + x >= 0 && midX + x < height && midY - y >= 0 && MY - y < width)
			space[(midX + x) * width + midY - y]++;
	
		if(midX - x >= 0 && midX - x < height && midY + y >= 0 && MY + y < width)
			space[(midX - x) * width + midY + y]++;
	
		if(midX - x >= 0 && midX - x < height && midY - y >= 0 && MY - y < width)
			space[(midX - x) * width + midY - y]++;
	
		if(midX + y >= 0 && midX + y < height && midY + x >= 0 && MY + x < width)
			space[(midX + y) * width + midY + x]++;
	
		if(midX + y >= 0 && midX + y < height && midY - x >= 0 && MY - x < width)
			space[(midX + y) * width + midY - x]++;
	
		if(midX - y >= 0 && midX - y < height && midY + x >= 0 && MY + x < width)
			space[(midX - y) * width + midY + x]++;
	
		if(midX - y >= 0 && midX - y < height && midY - x >= 0 && MY - x < width)
			space[(midX - y) * width + midY - x]++;
	
		if(d < 0){ 
			d = d + (4 * x) + 6; 
		}
		else{
			d = d + 4 * (x - y) + 10;
			y = y - 1;
	
		}
		x = x + 1;
	}
}

void FilterHoughCalib::DrawCircle(int midX, int midY, int rad, unsigned char * space, int width, int height){



	int d = 3 - (2 * rad);
	int x = 0;
	int y = rad;

	space[midX * width + midY] = 128;

	while(y > x){

		if(midX + x >= 0 && midX + x < height && midY + y >= 0 && MY + y < width)
			space[(midX + x) * width + midY + y] = 128;
	
		if(midX + x >= 0 && midX + x < height && midY - y >= 0 && MY - y < width)
			space[(midX + x) * width + midY - y] = 128;
	
		if(midX - x >= 0 && midX - x < height && midY + y >= 0 && MY + y < width)
			space[(midX - x) * width + midY + y] = 128;
	
		if(midX - x >= 0 && midX - x < height && midY - y >= 0 && MY - y < width)
			space[(midX - x) * width + midY - y] = 128;
	
		if(midX + y >= 0 && midX + y < height && midY + x >= 0 && MY + x < width)
			space[(midX + y) * width + midY + x] = 128;
	
		if(midX + y >= 0 && midX + y < height && midY - x >= 0 && MY - x < width)
			space[(midX + y) * width + midY - x] = 128;
	
		if(midX - y >= 0 && midX - y < height && midY + x >= 0 && MY + x < width)
			space[(midX - y) * width + midY + x] = 128;
	
		if(midX - y >= 0 && midX - y < height && midY - x >= 0 && MY - x < width)
			space[(midX - y) * width + midY - x] = 128;
	
		if(d < 0){ 
			d = d + (4 * x) + 6; 
		}
		else{
			d = d + 4 * (x - y) + 10;
			y = y - 1;
	
		}
		x = x + 1;
	}


}

