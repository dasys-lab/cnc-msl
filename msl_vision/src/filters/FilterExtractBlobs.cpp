/*
 * $Id: FilterExtractBlobs.cpp 2142 2007-04-15 10:49:00Z jewollen $
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
#include "FilterExtractBlobs.h"

#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>



FilterExtractBlobs::FilterExtractBlobs(int width, int height):Filter(OF_GRAY, width, height){


	rep = (short *) malloc(NREPS * sizeof(short));

	count = (int *) malloc(NREPS * sizeof(int));

	rois = (ROI *) malloc(NREPS * sizeof(ROI));

	
	init();
	
	

}



FilterExtractBlobs::~FilterExtractBlobs(){

	cleanup();

}
		

unsigned char * FilterExtractBlobs::process(unsigned char * src, unsigned int width, unsigned int height, std::vector<ROI>& retROIs, unsigned char blobColor, std::vector<BlobBounds> & blobs, DistanceLookupHelper & helper, int countThreshold){

	unsigned char * tgt = outputBuffer;

	double * LookupTable = helper.getLookupTable();

	retROIs.clear();
	blobs.clear();

	for(short i = 0; i < NREPS; i++){
		rep[i] = i;
	}

	bzero(count, NREPS*sizeof(int));
	bzero(rois, NREPS*sizeof(ROI));

	short * out = (short *) malloc(width*height*sizeof(short));
	memset(out,0,width*height*sizeof(unsigned short));

	out[0] = 1;
	count[0] = 1;
	short numberOfReps = 1;
	rois[0].top = 0;
	rois[0].bottom = 0;
	rois[0].left = 0;
	rois[0].right = 0;

	unsigned char currColor = 0;
	short leftLabel = 0;
	short leftRep = 0;
	short topLabel = 0;
	short topRep = 0;

	unsigned char left = 0;
	unsigned char top = 0;

	int counter1 = 0;
	int counter2 = 0;
	int counter3 = 0;
	int counter4 = 0;
	int counter5 = 0;
	int counter6 = 0;



	for(unsigned int j = 1; j < width; j++){

		currColor = src[j];

		if(currColor == blobColor){

			leftLabel = out[j-1];
			leftRep = rep[leftLabel];
			
			while(leftRep != rep[leftRep])
				leftRep = rep[leftRep];
	
			left = src[j-1];
	
			if(currColor == left){
				out[j] = leftLabel;
				count[leftRep]++;
				rois[leftRep].right = j;
			}
			else {
				numberOfReps++;
				out[j] = numberOfReps;
				count[numberOfReps] = 1;
				rois[numberOfReps].top = 0;
				rois[numberOfReps].bottom = 0;
				rois[numberOfReps].left = j;
				rois[numberOfReps].right = j;
	
			}

		}

	}


	for(unsigned int i = 1; i < height; i++){

		currColor = src[i*width];

		if(currColor == blobColor){

			topLabel = out[(i-1)*width];
	
			topRep = rep[topLabel];
	
			while(topRep != rep[topRep])
				topRep = rep[topRep];
	
	
			top = src[(i-1)*width];
	
	
			if(currColor == top){
	
				out[i*width] = topLabel;
				count[topRep]++;
				rois[topRep].bottom = i;
	
			}
			else {
				
				numberOfReps++;
				out[i*width] = numberOfReps;
				rois[numberOfReps].top = i;
				rois[numberOfReps].bottom = i;
				rois[numberOfReps].left = 0;
				rois[numberOfReps].right = 0;
	
	
			}

		}


		for(unsigned int j = 1; j < width; j++){



			currColor = src[i*width + j];

			if(currColor == blobColor){
	
		
				topLabel = out[(i-1)*width + j];
				topRep = rep[topLabel];
	
				while(topRep != rep[topRep])
					topRep = rep[topRep];
	
	
				leftLabel = out[i*width + (j - 1)];
				leftRep = rep[leftLabel];
	
				while(leftRep != rep[leftRep])
					leftRep = rep[leftRep];
	
				left = src[i*width + j - 1];//means[leftRep];
				top = src[(i-1)*width + j];//means[topRep];
	
	
				if(currColor == left){
	
					if(currColor == top){
	
						if(leftRep == topRep){
							out[i*width + j] = leftRep;
							count[leftRep]++;
							counter1++;
							//rois[leftRep].bottom = i;
							//rois[leftRep].right = (rois[leftRep].right > j ? rois[leftRep].right : j);
						}
						else{
							if(leftRep < topRep){
								out[i*width + j] = leftRep;
								rep[topRep] = leftRep;
								count[leftRep] += count[topRep] + 1;
	
								rois[leftRep].top = (rois[leftRep].top < rois[topRep].top ? rois[leftRep].top : rois[topRep].top);
								rois[leftRep].bottom = i; //(rois[leftRep].bottom > rois[topRep].bottom ? rois[leftRep].bottom : rois[topRep].bottom);
								rois[leftRep].left = (rois[leftRep].left < rois[topRep].left ? rois[leftRep].left : rois[topRep].left); 
								rois[leftRep].right = (rois[leftRep].right > rois[topRep].right ? rois[leftRep].right : rois[topRep].right);
	
								//rois[leftRep].top = (rois[leftRep].top < i ? rois[leftRep].top : i);
								//rois[leftRep].bottom = (rois[leftRep].bottom > i ? rois[leftRep].bottom : i);
								//rois[leftRep].left = (rois[leftRep].left < j ? rois[leftRep].left : j); 
								//rois[leftRep].right = (rois[leftRep].right > j ? rois[leftRep].right : j);
								counter2++;
	
							}
							else {
							
								out[i*width + j] = topRep;
								rep[leftRep] = topRep;
								count[topRep] += count[leftRep] + 1;
	
								rois[topRep].top = (rois[leftRep].top < rois[topRep].top ? rois[leftRep].top : rois[topRep].top);
								rois[topRep].bottom = i; //(rois[leftRep].bottom > rois[topRep].bottom ? rois[leftRep].bottom : rois[topRep].bottom);
								rois[topRep].left = (rois[leftRep].left < rois[topRep].left ? rois[leftRep].left : rois[topRep].left); 
								rois[topRep].right = (rois[leftRep].right > rois[topRep].right ? rois[leftRep].right : rois[topRep].right);
	
								//rois[topRep].top = (rois[topRep].top < i ? rois[topRep].top : i);
								//rois[topRep].bottom = (rois[topRep].bottom > i ? rois[topRep].bottom : i);
								//rois[topRep].left = (rois[topRep].left < j ? rois[topRep].left : j); 
								//rois[topRep].right = (rois[topRep].right > j ? rois[topRep].right : j);
								
								counter3++;
	
							}
						}
	
					}
					else {
	
						out[i*width + j] = leftRep;
						count[leftRep]++;
	
						//rois[leftRep].bottom = i;
						rois[leftRep].right = (rois[leftRep].right > (int) j ? rois[leftRep].right : (int) j);
						counter4++;
					}
				}
				else {
	
					if(currColor == top){
						out[i*width + j] = topRep;
						count[topRep]++;
						
						rois[topRep].bottom = i;
						//rois[topRep].right = (rois[topRep].right > j ? rois[topRep].right : j); 
	
						counter5++;
					}
					else {
						numberOfReps++;
						out[i*width + j] = numberOfReps;
						count[numberOfReps] = 1;
						
						rois[numberOfReps].top = i;
						rois[numberOfReps].bottom = i;
						rois[numberOfReps].left = j;
						rois[numberOfReps].right = j;
						counter6++;
					}
	
				}
			}
		}

	}

	int regionCount = 0;
	int numberOfSmallRegions = 0;
	int numberOfLargeRegions = 0;
	int numberOfValidRegions = 0;
	unsigned char repMap[NREPS];
	unsigned char valid[NREPS];
	bzero(valid, NREPS);

	for(int i = 1; i < numberOfReps; i++){
		if(rep[i] == i){

			int rheight = abs(rois[i].top - rois[i].bottom) + 1;
			int rwidth = abs(rois[i].left - rois[i].right) + 1;

			double ratio = (double) rheight / (double) rwidth;
			if(ratio < 1)
				ratio = 1.0/ratio;



			regionCount++;
			if(count[i] < 15)
				numberOfSmallRegions++;
			else if(count[i] > countThreshold)
				numberOfLargeRegions++;
			else if(ratio < 2.5 && rwidth*rheight*0.50 < count[i]){
				valid[i] = 1;
				numberOfValidRegions++;
				repMap[i] = numberOfValidRegions;

				//printf("Valid ROI %d %d %d %d %d\n", rois[i].top, rois[i].bottom, rois[i].left, rois[i].right, count[i]);

				for(int k = rois[i].top; k <= rois[i].bottom; k++){
					if(k*width + rois[i].left < 0 || k*width + rois[i].left >= width*height ||
						k*width + rois[i].right < 0 || k*width + rois[i].right >= width*height){

					}
					else {
						src[k*width + rois[i].left] = 128;
						src[k*width + rois[i].right] = 128;
					}

				}

				for(int k = rois[i].left; k <= rois[i].right; k++){
					if(rois[i].top*width + k < 0 || rois[i].top*width + k >= width*height ||
						rois[i].bottom*width + k < 0 || rois[i].bottom*width + k >= width*height){

					}
					else {
						src[rois[i].top*width + k] = 128;
						src[rois[i].bottom*width + k] = 128;
					}

				}


				retROIs.push_back(rois[i]);

				BlobBounds validBlob;
				validBlob.left = rois[i].left;
				validBlob.right = rois[i].right;
				validBlob.top = rois[i].top;
				validBlob.bottom = rois[i].bottom;
				validBlob.minX = rois[i].top;
				validBlob.minY = rois[i].left;
				validBlob.count = 0;
				validBlob.minDistance = 30000.0;

				for(int k = rois[i].top; k <= rois[i].bottom; k++){
					for(int l = rois[i].left; l <= rois[i].right; l++){

						if(src[k*width + l] == blobColor){
							double distance = LookupTable[k*width + l];
							distance = (distance > 0.0 ? distance : 30000.0);
							if(validBlob.minDistance > distance){
								////printf("minDistanceChanged: %f\n", distance);
								validBlob.minX = k;
								validBlob.minY = l;
								validBlob.minDistance = distance;
			
							}
							validBlob.count++;
						}

					}
				}

				blobs.push_back(validBlob);


			}
		}

	}


/*	for(int i = 0; i < width*height; i++){
		short currRep = out[i];
		while(currRep != rep[currRep])
			currRep = rep[currRep];
		if(valid[currRep] > 0)
			tgt[i] = repMap[currRep] + 80;
		else
			tgt[i] = 0;


	}*/

	//printf("Found %d regions out of %d representatives\n", regionCount, numberOfReps);
	//printf("Number of small regions: %d\n", numberOfSmallRegions);
	//printf("Number of large regions: %d\n", numberOfLargeRegions);
	//printf("Number of valid regions: %d\n", numberOfValidRegions);
	//printf("Counters: %d %d %d %d %d %d\n", counter1, counter2, counter3, counter4, counter5, counter6);

	free(out);	

	return tgt;

}



void FilterExtractBlobs::init(){


}


void FilterExtractBlobs::cleanup(){
	
	if(rep != NULL)
		free(rep);
	if(count != NULL)
		free(count);
	if(rois != NULL)
		free(rois);


}

