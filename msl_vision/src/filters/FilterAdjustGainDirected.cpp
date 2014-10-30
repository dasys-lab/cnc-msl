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
#include "FilterAdjustGainDirected.h"

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

using namespace std;

FilterAdjustGainDirected::FilterAdjustGainDirected(int _width, int _height, int g_gain, int g_exposure):Filter(OF_GRAY, _width, _height){
	gain = g_gain;
	exposure = g_exposure;
	init();

}


FilterAdjustGainDirected::~FilterAdjustGainDirected(){
	cleanup();
}
		


int FilterAdjustGainDirected::process(CameraQuickCam *camera, unsigned char * src, int avgInt, int iwidth, int iheight) {

	int length = iwidth*iheight;

	long long ret=0;
	long long pixs=0;
	const int lowerBound = 50;

	cout << "ASD" << iwidth << " " << (iwidth-lowerBound)*iheight << endl;

	for(int index = 0; index < length; index++){
		if(index % iwidth > lowerBound) continue;
		ret += src[index];
		//pixs++;
	}		
	//ret = ret / ((long long) ((iwidth-lowerBound)*iheight));
	ret = ret / ((long long) ((lowerBound)*iheight));

	if(ret>50) {

		if(exposure > 220) {
			exposure -= 5;
			camera->set_exposure(exposure);
		}
		else {
			gain--;
			if(gain<0) {
				gain = 0;
				exposure -= 5;
				camera->set_exposure(exposure);
			}
			camera->set_gain(gain);
		}
	}
	else if(ret<30) {
		if(exposure < 125) {
			exposure += 5;
			camera->set_exposure(exposure);
		}
		else {
			gain++;
			if(gain>50) {
				gain = 50;
				exposure += 5;
				camera->set_exposure(exposure);
			}
			camera->set_gain(gain);
		}
	}

	std::cout << "AvgIntensity: " << ret  << " Gainlevel: " << gain << " Exposure: " << exposure << std::endl;

	return ret;
}



void FilterAdjustGainDirected::init(){
}


void FilterAdjustGainDirected::cleanup(){
}

