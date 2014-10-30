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
#include "FilterYUVCountColoredDots.h"

#include <stdio.h>
#include <string>
#include <iostream>
#include <math.h>

//#include "floatfann.h"



FilterYUVCountColoredDots::FilterYUVCountColoredDots(int width_, int height_, int refx_, int refy_, int areaWidth_, int areaHeight_):Filter(OF_ZERO, width_, height_){


	width = width_;
	height = height_;
	
	refx = refx_;
	refy = refy_;

	areaWidth = areaWidth_;
	areaHeight = areaHeight_;

	lookupTable = (unsigned char *) malloc(256*256);

	if(lookupTable == NULL)
		printf("lookupTable = null!!!\n");
	
	init();

}



FilterYUVCountColoredDots::~FilterYUVCountColoredDots(){

	cleanup();

}
		



long long FilterYUVCountColoredDots::process(unsigned char * src){
	long long ret=0;	


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
			color = lookupTable[u*256 + v];
	
			if(j<(areaWidth/2)) continue;
			if(color > 100) ret++;
		}
	}
	return ret;

}



void FilterYUVCountColoredDots::init(){
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

/*
			int value = (int) lrint(((cos(angle)*(u-center) + sin(angle)*(v-center)) * 128)*2.0);
			double diffAngle = fabs(atan2(v-center, u-center) - angle);
			double distance = sqrt((v-center)*(v-center) + (u-center)*(u-center))/128.0;
			if(diffAngle > M_PI)
				diffAngle = fabs(diffAngle - 2.0*M_PI);
			value = (int) 255 - lrint(2.5*diffAngle*180.0/M_PI*(0.5*distance + 0.5));
			value = lrint(distance*255*1.5);
			//
                        int value2 = (int)lrint((pow(v,1.81) - u)/64.0);
                        value2 -= (int) lrint(0.05*diffAngle*180.0/M_PI);

			//value = (value2/2) + (value/2);
			if(value < 0)
				value = 0;
			if(value > 255)
				value = 255;
*/
                        int value = (int) lrint(((cos(angle)*(u-center) + sin(angle)*(v-center)) * 128)*2.0);
                        double diffAngle = fabs(atan2(v-center, u-center) - angle);
                        double distance = sqrt((fabs(220-v))*(fabs(220-v)) + (fabs(50-u))*(fabs(50-u)));
                        if(diffAngle > M_PI)
                                diffAngle = fabs(diffAngle - 2.0*M_PI);
                        value = (int) 255.0 - lrint(5.0*diffAngle*180.0/M_PI*(0.5*distance + 0.5));
                        value = 255-lrint(3*distance);


                        int value2 = (int)lrint((pow(v,1.75) - u)/64.0);
                        value2 -= (int) lrint(0.5*diffAngle*180.0/M_PI);
                        value = ((value2/2) + (value/2))/1.7;
                        //value = value2;
                        if(value < 50)
                        	value = 0;
                        if(value > 55)
                                value = 255;
			lookupTable[u*256 + v] = (unsigned char) value; 
		}
	}


}

void FilterYUVCountColoredDots::cleanup(){
	
	free(lookupTable);

}

