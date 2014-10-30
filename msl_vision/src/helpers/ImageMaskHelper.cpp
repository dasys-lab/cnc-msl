/*
 * $Id: ImageMaskHelper.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "ImageMaskHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>

ImageMaskHelper::ImageMaskHelper(){

	printf("Constructor of ImageMaskHelper\n");
	LookupTable = NULL;

	imWidth = 640;
	imHeight = 480;
	
	init();


}


ImageMaskHelper::ImageMaskHelper(int width, int height){

	printf("Constructor of ImageMaskHelper\n");
	LookupTable = NULL;

	imWidth = width;
	imHeight = height;
	
	init();


}



ImageMaskHelper::~ImageMaskHelper(){

	cleanup();

}



void ImageMaskHelper::init(){

/*	
	if(LookupTable == NULL){
		LookupTable = (unsigned char *) malloc(imWidth*imHeight*sizeof(unsigned char));
	}

	std::string file_name = std::string(getenv("ES_ROOT")) + "/src/Vision5/" + std::string(MASKFILENAME);


	FILE * fd = fopen(file_name.c_str(), "r");
	if(fd != NULL){
		fread(LookupTable, sizeof(unsigned char), imWidth*imHeight, fd);
		fclose(fd);
	}
	else{
		printf("ImageMaskHelper: File not found ....\n");
		printf("file name: %s\n", file_name.c_str());
		exit(1);
	}

*/
/*	FILE * fd2 = fopen("Karotte.txt", "w");

	for(int i = 0; i < HEIGHT; i++){
		for(int j = 0; j < WIDTH; j++){
			fprintf(fd2, "%d ", LookupTable[i*WIDTH + j]);
		}
		fprintf(fd2, "\n");
	}

	fclose(fd2); */

}

void ImageMaskHelper::cleanup(){

//	if(LookupTable != NULL)
//		free(LookupTable);

}


unsigned char * ImageMaskHelper::getLookupTable(){

//	return LookupTable;

}

