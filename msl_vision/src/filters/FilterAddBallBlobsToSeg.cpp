/*
 * $Id: FilterAddBallBlobsToSeg.cpp 2142 2007-04-15 10:49:00Z jewollen $
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
#include "FilterAddBallBlobsToSeg.h"

#include <stdio.h>
#include <string>
#include <iostream>




FilterAddBallBlobsToSeg::FilterAddBallBlobsToSeg(int width, int height):Filter(OF_ZERO, width, height){
	
	init();

}



FilterAddBallBlobsToSeg::~FilterAddBallBlobsToSeg(){

	cleanup();

}
		

unsigned char * FilterAddBallBlobsToSeg::process(unsigned char * src, unsigned char * tgt, std::vector<BlobBounds> & blobs, unsigned int width, unsigned int height, unsigned char addColor){

	for(unsigned int a = 0; a < blobs.size(); a++){
		for(int i = blobs[a].top; i <= blobs[a].bottom; i++){
			for(int j = blobs[a].left; j <= blobs[a].right; j++){
				if(src[i*width + j] == addColor)
					tgt[i*width + j] = COLOR_RED;


			}
		}

	}

	return tgt;

}



void FilterAddBallBlobsToSeg::init(){


}


void FilterAddBallBlobsToSeg::cleanup(){
	
	//delete sharedMemoryHelper;

}

