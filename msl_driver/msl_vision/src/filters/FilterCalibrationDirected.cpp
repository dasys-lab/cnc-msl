/*
 * $Id: FilterCalibrationDirected.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "FilterCalibrationDirected.h"
#include "../helpers/PositionHelperDirected.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

FilterCalibrationDirected::FilterCalibrationDirected(int width, int height):Filter(OF_ZERO, width, height){

	imWidth = width;
	imHeight = height;

	init();

}



FilterCalibrationDirected::~FilterCalibrationDirected(){

	cleanup();

}
		

unsigned char * FilterCalibrationDirected::process(unsigned char * src, int width, int height){


	unsigned char * tgt = src;

	PositionHelperDirected * helper = PositionHelperDirected::getInstance();

	for(int i = 0; i < height; i++){

		for(int j = 0; j < width; j++){

			double dist = helper->getPointCam2Dist(j, i)/500.0;

			if(dist > 0 && dist < 10000.0 && fabs(dist - trunc(dist)) < 0.30)
				tgt[i*width + j] = 0;

		}
	}

	return tgt;


}



void FilterCalibrationDirected::init(){


}


void FilterCalibrationDirected::cleanup(){


}


