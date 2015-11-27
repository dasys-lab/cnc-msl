/*
 * $Id: DistanceLookupHelper.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "DistanceLookupHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>

DistanceLookupHelper * DistanceLookupHelper::instance_ = NULL;

DistanceLookupHelper::DistanceLookupHelper(int area){

	printf("Constructor of DistanceLookupHelper\n");
	LookupTable = NULL;
	LookupTableInt = NULL;
	HorizontalLookupTable = NULL;
	imWidth = area;
	imHeight = area;

	mx = area/2;
	my = area/2;

	init(FILENAME);
	
	instance_ = this;


}

DistanceLookupHelper::DistanceLookupHelper(char* filename, int area){

	printf("Constructor of DistanceLookupHelper\n");
	LookupTable = NULL;
	LookupTableInt = NULL;
	HorizontalLookupTable = NULL;
	imWidth = area;
	imHeight = area;

	mx = area/2;
	my = area/2;

	init(filename);

	instance_ = this;

}


DistanceLookupHelper::DistanceLookupHelper(char* filename, int areaWidth, int areaHeight){

	printf("Constructor of DistanceLookupHelper Correct\n");
	LookupTable = NULL;
	LookupTableInt = NULL;
	HorizontalLookupTable = NULL;
	imWidth = areaWidth;
	imHeight = areaHeight;

	mx = areaHeight/2;
	my = areaWidth/2;

	init(filename);

	instance_ = this;

}



DistanceLookupHelper::DistanceLookupHelper(char* filename, short mx_, short my_){
	printf("Constructor of DistanceLookupHelper\n");
	LookupTable = NULL;
	LookupTableInt = NULL;
	HorizontalLookupTable = NULL;
	mx = mx_;
	my = my_;

	imWidth = 640;
	imHeight = 480;


	init(filename);

	instance_ = this;

}

DistanceLookupHelper::DistanceLookupHelper(short mx_, short my_){

	printf("Constructor of DistanceLookupHelper\n");
	LookupTable = NULL;
	LookupTableInt = NULL;
	HorizontalLookupTable = NULL;
	mx = mx_;
	my = my_;

	imWidth = 640;
	imHeight = 480;


	init(FILENAME);

	instance_ = this;

}


DistanceLookupHelper::~DistanceLookupHelper(){

	cleanup();

}



void DistanceLookupHelper::init(char* name){

	
	if(LookupTable == NULL){
		LookupTable = (double *) malloc(imWidth*imHeight*sizeof(double));
	}
	if(LookupTableInt == NULL){
		LookupTableInt = (int *) malloc(imWidth*imHeight*2*sizeof(int));
	}

	if(HorizontalLookupTable == NULL){

		HorizontalLookupTable = (double *) malloc(HLOOKUPSIZE*sizeof(double));
	}

	std::string file_name = std::string(getenv("DOMAIN_CONFIG_FOLDER")) +"/"+ std::string(name);

	FILE * fd = fopen(file_name.c_str(), "r");
	if(fd != NULL){
		fread(LookupTable, sizeof(double), imWidth*imHeight, fd);
		fread(LookupTableInt, sizeof(int), imWidth*imHeight*2, fd);
		fclose(fd);
	}
	else{
		printf("Distance Helper Lookup: File not found ....\n");
		printf("file name: %s\n", file_name.c_str());
		exit(1);
	}


	for(short i = 0; i < HLOOKUPSIZE; i++){
		
		int x = mx;
		int y = my + i;
		
		//printf("HLT: %d %d %d %f\n", x, y, i, LookupTable[x*imWidth + y]);

		HorizontalLookupTable[i] = LookupTable[x*imWidth + y];
		
		
	}

}

void DistanceLookupHelper::cleanup(){

	if(LookupTable != NULL)
		free(LookupTable);
	if(LookupTableInt != NULL)
		free(LookupTableInt);
	if(HorizontalLookupTable != NULL)
		free(HorizontalLookupTable);

}


double * DistanceLookupHelper::getLookupTable(){

	return LookupTable;

}

int * DistanceLookupHelper::getLookupTableInt(){

	return LookupTableInt;

}

double * DistanceLookupHelper::getHorizontalLookupTable(){

	return HorizontalLookupTable;

}

DistanceLookupHelper * DistanceLookupHelper::getCreatedInstance(){

	if(instance_ == NULL){
		printf("DistanceLookupHelper: tried to access instance, but not created!");
		exit(1);
	}

	return instance_;

}

