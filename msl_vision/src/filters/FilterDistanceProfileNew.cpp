/*
 * $Id: FilterDistanceProfileNew.cpp 1935 2007-03-19 19:50:12Z phbaer $
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
#include "FilterDistanceProfileNew.h"

#include <stdio.h>
#include <stdlib.h>
#include <msl_sensor_msgs/DistanceScanInfo.h>
#include <math.h>


#include <msl_msgs/Point2dInfo.h>
#include <msl_sensor_msgs/ObstacleInfo.h>

#include "../helpers/Lookuptable.h"
#include "../helpers/SpicaHelper.h"

using namespace msl_sensor_msgs;


FilterDistanceProfileNew::FilterDistanceProfileNew(int width, int height):Filter(OF_ZERO, width, height){

	this->sc = SystemConfig::getInstance();

	profile = NULL;
	tmpProfile = NULL;

	Configuration *vision = (*this->sc)["Vision"];

	minObsDistance = vision->tryGet<short>(0 ,"Vision", "MinObsDistance", NULL);
	printf("minObsDistance %d", minObsDistance);
	obsThreshOffset = vision->tryGet<short>(50 ,"Vision", "ObsThreshOffset", NULL);

	negRanges[0][0] = vision->get<short>("Vision", "Holder", "NegRange_0_0", NULL);
	negRanges[0][1] = vision->get<short>("Vision", "Holder", "NegRange_0_1", NULL);
	negRanges[1][0] = vision->get<short>("Vision", "Holder", "NegRange_1_0", NULL);
	negRanges[1][1] = vision->get<short>("Vision", "Holder", "NegRange_1_1", NULL);
	negRanges[2][0] = vision->get<short>("Vision", "Holder", "NegRange_2_0", NULL);
	negRanges[2][1] = vision->get<short>("Vision", "Holder", "NegRange_2_1", NULL);
	numberOfLines = -1;

	shared_ptr<std::vector<std::string> > holdersBPtr = (*vision).getSections("Vision", "Holder", NULL);
	std::vector<std::string> * holders = holdersBPtr.get();

	addHolders.clear();

	for(unsigned int i = 0; i < holders->size(); i++){

		Holder addHolder;
		addHolder.start = vision->get<double>("Vision", "Holder", (*holders)[i].c_str(), "Start", NULL);
		addHolder.end = vision->get<double>("Vision", "Holder", (*holders)[i].c_str(), "End", NULL);
		addHolders.push_back(addHolder);

		printf("Holder: %s %f %f\n", (*holders)[i].c_str(), addHolder.start, addHolder.end);

	}

	init();

}



FilterDistanceProfileNew::~FilterDistanceProfileNew(){

	cleanup();

}

double FilterDistanceProfileNew::getThreshold() {
	return threshold_;
}		

unsigned char * FilterDistanceProfileNew::process(unsigned char * src, unsigned int width, unsigned int height, ScanLineHelper & helper, DistanceLookupHelper & distanceHelper){

	
	unsigned char jumpThreshold = 5;
	unsigned char limitThres = 50;
	unsigned char lineThres = 70;

	unsigned char * tgt = src;

	numberOfLines = helper.getNumberLines();

	double * LookupTable = distanceHelper.getLookupTable();

	//printf("Address of DistanceHelper: %ld\n", &distanceHelper);

	if(profile == NULL){
		profile = (double *) malloc(NSECTORS*sizeof(double));
	}
	for(int i = 0; i < NSECTORS; i++){
		profile[i] = HORIZON;
	}


	if(tmpProfile == NULL){
		tmpProfile = (double *) malloc(helper.getNumberLines()*sizeof(double));
	}
	for(int i = 0; i < helper.getNumberLines(); i++){

		tmpProfile[i] = HORIZON;
	}

	double * tmpProfileShrinked = (double *) malloc(helper.getNumberLines()/2*sizeof(double));

	for(int i = 0; i < helper.getNumberLines()/2; i++){
		tmpProfileShrinked[i] = HORIZON;
	}

	double * tmpProfileShrinkedCopy = (double *) malloc(helper.getNumberLines()/2*sizeof(double));

	for(int i = 0; i < helper.getNumberLines()/2; i++){
		tmpProfileShrinkedCopy[i] = HORIZON;
	}

///////////////Calculate Threshold/////////////////////
	unsigned int mx = height/2;
	unsigned int my = width/2;

	unsigned int frame = 25; //25;

	unsigned int sum = 0;	

	for(unsigned int i = mx - frame; i < mx + frame; i++){
		for(unsigned int j = my - frame; j < my + frame; j++){
						
			sum += src[i*width + j];
		
		}
	}
			
	double threshold = ((double)sum)/((double)(4*frame*frame));

	double addThreshold = threshold;
	if(addThreshold < 3.0)
		addThreshold = 3.0;
	if(addThreshold > 10.0)
		addThreshold = 10.0;

	threshold += addThreshold;
	threshold_ = threshold;

	//printf("FilterGrayToSeg: threshold = %f\n", threshold);

	unsigned char threshold_ind = (unsigned char) lrint(threshold);
	if(threshold_ind < 10)
		threshold_ind = 10;

////////////////Threshold Calculated/////////////////////////////

	unsigned char luminances[4];
	for(int i = 0; i < 4; i++)
		luminances[i] = 0;
	short regionFinder = helper.getNumberLines()/4;


////////////////Calculate Luminance Regions//////////////////////


	short * firstInnerLum = helper.getInnerLines();
	short * nInnerLum = helper.getInnerLinesN();

	////printf("Number of ScanLines: %d\n", helper.getNumberLines());

	short maxPointsLum = helper.getMaxPoints();

	short xLum;
	short yLum;


	for(short i = 0; i < helper.getNumberLines(); i++){

		short region = i/regionFinder;

		if(i % 2 == 0){

			short * line = firstInnerLum;

			for(short j = 0; j < (*nInnerLum); j++){

				xLum = *line++;
				yLum = *line++;

				unsigned char currColor = src[xLum*width + yLum];
				if(currColor > luminances[region])
					luminances[region] = currColor;

				if(LookupTable[xLum*width + yLum] > 7000.0)
					break;

			}

			firstInnerLum = firstInnerLum + maxPointsLum*2;
			nInnerLum++;
	
		}

	}

	//printf("Luminances: %d %d %d %d\n", luminances[0], luminances[1], luminances[2], luminances[3]);


//////////////////////////////Luminance Regions Calculated////////////////////////////



	short * firstInner = helper.getInnerLines();
	short * nInner = helper.getInnerLinesN();

	short * firstOuter = helper.getOuterLines();
	short * nOuter = helper.getOuterLinesN();

	////printf("Number of ScanLines: %d\n", helper.getNumberLines());

	short maxPoints = helper.getMaxPoints();

	short x;
	short y;


	for(short i = 0; i < helper.getNumberLines(); i++){

		bool found = false;
		short counter = 0;
		short indX = 0;
		short indY = 0;

		short region = i/regionFinder;		

		short luminanceAdd = (((short) luminances[region]) - 100)/10;
		if(luminanceAdd < -5)
			luminanceAdd = -5;

		unsigned char threshold_ind_new = (unsigned char) (max((short) threshold_ind + luminanceAdd + obsThreshOffset, 0));



		if(i % 2 == 0){

			short * line = firstInner;
			unsigned char colorBefore = 0;
			unsigned char colorBeforeBefore = 0;
			unsigned char threshold_float = threshold_ind;

			bool afterJump = false;

			for(short j = 0; j < (*nInner); j++){

				x = *line++;
				y = *line++;
				//uncomment to avoid own shadows
				if(j<minObsDistance) continue;

				unsigned char currColor = src[x*width + y];

				if(currColor <= limitThres || (currColor <= threshold_ind_new && currColor + jumpThreshold <= colorBefore && colorBefore < lineThres && colorBeforeBefore < lineThres))
					afterJump = true;

				if(afterJump && currColor <= threshold_ind_new){// || currColor <= threshold_float || (currColor + jumpThreshold <= colorBefore && currColor < 60)){
					if(currColor + jumpThreshold <= colorBefore && currColor < 60)
						threshold_float = currColor + 3;
					counter++;

					if(counter == 1){
						indX = x;
						indY = y;
					}
					if(counter >= RANGE){
						found = true;
						src[indX*width + indY] = 255;
						DrawCircle(indX, indY, 5, src, width, height);

						break;
					}
				}
				else{
					threshold_float = threshold_ind;
					counter = 0;
					afterJump = false;
				}
				colorBeforeBefore = colorBefore;
				colorBefore = currColor;
			}

			firstInner = firstInner + maxPoints*2;
			nInner++;
	
		}
		else {

			short * line = firstOuter;
			unsigned char colorBefore = 0;
			unsigned char colorBeforeBefore = 0;
			unsigned char threshold_float = threshold_ind;

			bool afterJump = false;

			for(short j = 0; j < (*nOuter); j++){

				x = *line++;
				y = *line++;

				unsigned char currColor = src[x*width + y];

				if(currColor <= limitThres || (currColor <= threshold_ind_new && currColor + jumpThreshold <= colorBefore && colorBefore < lineThres && colorBeforeBefore < lineThres))
					afterJump = true;


				if(afterJump && currColor <= threshold_ind_new){// || currColor <= threshold_float || (currColor + jumpThreshold <= colorBefore && currColor < 60)){
					if(currColor + jumpThreshold <= colorBefore && currColor < 60)
						threshold_float = currColor + 3;
					counter++;

					if(counter == 1){
						indX = x;
						indY = y;
					}

					if(counter >= RANGE){
						found = true;
						src[indX*width + indY] = 255;
						DrawCircle(indX, indY, 5, src, width, height);
						break;
					}
				}
				else{
					counter = 0;
					threshold_float = threshold_ind;
					afterJump = false;
				}
				colorBeforeBefore = colorBefore;
				colorBefore = currColor;
			}

			firstOuter = firstOuter + maxPoints;
			nOuter++;

		}

		if(found){
			double calcDistance = LookupTable[indX*width + indY];
			if(calcDistance < 0.0)
				tmpProfile[i] = HORIZON;
			else{
				tmpProfile[i] = calcDistance;
			}

			if(negRanges[0][0] > helper.getNumberLines()/2 && negRanges[0][1] < helper.getNumberLines()/2){

				if( (i >= negRanges[0][0]) || (i <= negRanges[0][1]) || 
					(i >= negRanges[1][0] && i <= negRanges[1][1]) ||
					(i >= negRanges[2][0] && i <= negRanges[2][1]))
					tmpProfile[i] = HORIZON;

			}
			else {

				if( (i >= negRanges[0][0] && i <= negRanges[0][1]) || 
					(i >= negRanges[1][0] && i <= negRanges[1][1]) ||
					(i >= negRanges[2][0] && i <= negRanges[2][1]))
					tmpProfile[i] = HORIZON;

			}


			double lineAngle = i*1.0/helper.getNumberLines()*360.0;
			
			for(unsigned a = 0; a < addHolders.size(); a++){
				if(addHolders[a].start > 180.0 && addHolders[a].end < 180.0){
			
					if(lineAngle >= addHolders[a].start || lineAngle <= addHolders[a].end)
						tmpProfile[i] = HORIZON;
			
				}
				else {
			
					if(lineAngle >= addHolders[a].start && lineAngle <= addHolders[a].end)
						tmpProfile[i] = HORIZON;
			
				}
			
			}


		}
		else{
			tmpProfile[i] = HORIZON;

		}

	}


////////////////////////FillUpNegRanges///////////////////////////////

	//printf("FillUpNegRanges\n");


	short gap = 2;

	double minInRange = HORIZON;

	short currIndex0 = negRanges[0][0] - gap;
	if(currIndex0 < 0)
		currIndex0 = currIndex0 + helper.getNumberLines();

	short endIndex0 = negRanges[0][1] + gap;
	if(endIndex0 >= helper.getNumberLines())
		endIndex0 = endIndex0 - helper.getNumberLines();

	
	while(true){
		if(tmpProfile[currIndex0] < minInRange)
			minInRange = tmpProfile[currIndex0];

		if(currIndex0 == endIndex0)
			break;

		currIndex0++;
		if(currIndex0 >= helper.getNumberLines())
			currIndex0 = 0;
	}

	currIndex0 = negRanges[0][0];
	endIndex0 = negRanges[0][1];

	while(true){
		
		tmpProfile[currIndex0] = minInRange;

		if(currIndex0 == endIndex0)
			break;

		currIndex0++;
		if(currIndex0 >= helper.getNumberLines())
			currIndex0 = 0;
	}


	minInRange = HORIZON;

	for(short i = negRanges[1][0] - gap; i <= negRanges[1][1] + gap; i++){

		if(tmpProfile[i] < minInRange)
			minInRange = tmpProfile[i];


	}


	for(short i = negRanges[1][0]; i <= negRanges[1][1]; i++){

		tmpProfile[i] = minInRange;

	}


	minInRange = HORIZON;

	for(short i = negRanges[2][0] - gap; i <= negRanges[2][1] + gap; i++){


		if(tmpProfile[i] < minInRange)
			minInRange = tmpProfile[i];


	}


	for(short i = negRanges[2][0]; i <= negRanges[2][1]; i++){


		tmpProfile[i] = minInRange;

	}




	for(unsigned int a = 0; a < addHolders.size(); a++){

		minInRange = HORIZON;

		currIndex0 = (short) lrint(addHolders[a].start/360.0*helper.getNumberLines()) - gap;
		if(currIndex0 < 0)
			currIndex0 = currIndex0 + helper.getNumberLines();
	
		endIndex0 = (short) lrint(addHolders[a].end/360.0*helper.getNumberLines()) + gap;
		if(endIndex0 >= helper.getNumberLines())
			endIndex0 = endIndex0 - helper.getNumberLines();
	
		
		while(true){
			if(tmpProfile[currIndex0] < minInRange)
				minInRange = tmpProfile[currIndex0];
	
			if(currIndex0 == endIndex0)
				break;
	
			currIndex0++;
			if(currIndex0 >= helper.getNumberLines())
				currIndex0 = 0;
		}
	
		currIndex0 = (short) lrint(addHolders[a].start/360.0*helper.getNumberLines());
		if(currIndex0 >= helper.getNumberLines())
				currIndex0 = 0;
		endIndex0 = (short) lrint(addHolders[a].end/360.0*helper.getNumberLines());
		if(endIndex0 >= helper.getNumberLines())
				endIndex0 = 0;
	
		while(true){
			
			tmpProfile[currIndex0] = minInRange;
	
			if(currIndex0 == endIndex0)
				break;
	
			currIndex0++;
			if(currIndex0 >= helper.getNumberLines())
				currIndex0 = 0;
		}


	}


///////////////////////EndFillUpNegRanges/////////////////////////////


//////////////////////CalculateShrinkedProfile////////////////////////

	//printf("CalculateShrinkedProfile\n");

	for(short i = 0; i < helper.getNumberLines()/2; i++){

		double value = tmpProfile[2*i];
		if(tmpProfile[2*i + 1] < value)
			value = tmpProfile[2*i + 1];
		if(value > 6000.0)
			value = HORIZON;


		tmpProfileShrinked[i] = value;


	}

	memcpy((void *) tmpProfileShrinkedCopy, (void *) tmpProfileShrinked, helper.getNumberLines()/2*sizeof(double));

///////////////////////EndCalculateShrinkedProfile/////////////////////



///////////////////////CalculateOpponents//////////////////////////////

	//printf("CalculateOpponents\n");

	bool finished = false;
	gap = 1;
	double distThreshold = 1000.0;

	std::vector<Point> opponentPoints;
	opponentPoints.clear();

	while(!finished){
		
		double minValue = HORIZON;
		short minIndex = -1;

		for(short i = 0; i < helper.getNumberLines()/2; i++){
			if(tmpProfileShrinkedCopy[i] < minValue){
				minValue = tmpProfileShrinkedCopy[i];
				minIndex = i;
			}
		}

		if(minIndex == -1){
			finished = true;
			break;
		}

		Range range;
		range.start = minIndex;
		range.end = minIndex;
		range.minDistance = tmpProfileShrinkedCopy[minIndex];
		short rangeLengthChecked = 0;

		short index = minIndex;
		short gapCounter = 0;

		while(true){

			if(gapCounter > gap)
				break;

			index++;
			if(index >= helper.getNumberLines()/2)
				index = 0;
				
			if(tmpProfileShrinkedCopy[index] < distThreshold + tmpProfileShrinkedCopy[minIndex]){
				tmpProfileShrinkedCopy[index] = HORIZON;
				range.end = index;
				gapCounter = 0;
			}
			else{
				gapCounter++;
			}
			rangeLengthChecked++;

			if(rangeLengthChecked > helper.getNumberLines()/2){
				range.start = minIndex;
				range.end = minIndex;
				break;

			}
		}

		rangeLengthChecked = 0;

		index = minIndex;
		gapCounter = 0;

		while(true){

			if(gapCounter > gap)
				break;

			index--;
			if(index < 0)
				index = helper.getNumberLines()/2 - 1;
				
			if(tmpProfileShrinkedCopy[index] < distThreshold + tmpProfileShrinkedCopy[minIndex]){
				tmpProfileShrinkedCopy[index] = HORIZON;
				range.start = index;
				gapCounter = 0;
			}
			else{
				gapCounter++;
			}
			rangeLengthChecked++;

			if(rangeLengthChecked > helper.getNumberLines()/2){
				range.start = minIndex;
				range.end = minIndex;
				break;

			}
		}

		tmpProfileShrinkedCopy[minIndex] = HORIZON;

		short rangeLength = range.end - range.start;
		if(rangeLength < 0)
			rangeLength += helper.getNumberLines()/2;
		rangeLength++;

		double oppAngle = -4.0*minIndex;
		oppAngle = M_PI*oppAngle/180.0;
		if(oppAngle < -M_PI)
			oppAngle += 2.0*M_PI;

		//printf("Opponent Range detected: %d %d %d %f %f\n", minIndex, range.start, range.end, oppAngle, range.minDistance);

		double arcusLength = rangeLength*4.0/360.0*2.0*M_PI*range.minDistance;
		//printf("ArcusLength: %f\n", arcusLength);

		if(arcusLength < 250.0){
			short i = range.start;
			tmpProfileShrinked[i] = HORIZON;
			
			while(i != range.end){
				i++;
				if(i >= helper.getNumberLines()/2)
					i = 0;
				tmpProfileShrinked[i] = HORIZON;
				////printf("Filtered Out %d\n", i);
			}
		}
		else {

			calculateOpponentsFromRange(range, tmpProfileShrinked, helper.getNumberLines()/2, 4.0, opponentPoints);


		}


	}

	//printf("EndCalculateOppponents\n");

	//printf("Oppponents:\n");

	for(unsigned int i = 0; i < opponentPoints.size(); i++){

		double oppDist = sqrt(opponentPoints[i].x*opponentPoints[i].x + opponentPoints[i].y*opponentPoints[i].y);

		//printf("%f %f %f\n", opponentPoints[i].x, opponentPoints[i].y, oppDist);

	}


	SpicaHelper::wm->obstacles.clear();
	ObstacleInfo oi;
	for(unsigned int i = 0; i < opponentPoints.size(); i++){ 
		oi.x = opponentPoints[i].x;
		oi.y = opponentPoints[i].y;

		SpicaHelper::wm->obstacles.push_back(oi);

	}


///////////////////////EndCalculateOpponents///////////////////////////


	

	double currentMin = tmpProfile[0];
	short index = helper.getNumberLines()/2 - 1;
	short sector = 0;
	short nLines = helper.getNumberLines()/(2*NSECTORS);
	short counter = 1;

	//printf("NumberLines: %d\n", helper.getNumberLines());
	//printf("NSECTORS: %d\n", NSECTORS);
	//printf("Index: %d\n", index);
	//printf("nLines: %d\n", nLines);

	double nLines_double = helper.getNumberLines()/(2.0*NSECTORS);

	//printf("nLines_double: %f\n", nLines_double);

	//printf("Distance Profile:\n");

	while(index >= 0){

		double sector_double = index*1.0/nLines_double;
		sector = (short) lrint(sector_double);

		sector = (NSECTORS - 1) - sector; 

		if(sector >= NSECTORS){
			sector = NSECTORS - 1;
			//printf("Sector was too big\n");
		}
		if(sector < 0){
			sector = 0;
			//printf("Sector was too small\n");
		}

		

		if(profile[sector] > tmpProfileShrinked[index]){
			profile[sector] = tmpProfileShrinked[index];
			
		}

		/*//printf("%f ", profile[sector]);	
		if(profile[sector] < HORIZON)
			//printf("<HORIZON");*/

/*		if(counter == nLines){
			if(currentMin > tmpProfileShrinked[index])
				currentMin = tmpProfileShrinked[index];
			profile[sector] = currentMin;
			sector++;
			currentMin = tmpProfileShrinked[index];
			counter = 1;
		}
		else{
			if(currentMin > tmpProfileShrinked[index])
				currentMin = tmpProfileShrinked[index];
			counter++;	
		}*/
		index--;
	}

	////printf("\n");

	if(tmpProfileShrinked != NULL)
		free(tmpProfileShrinked);

	if(tmpProfileShrinkedCopy != NULL)
		free(tmpProfileShrinkedCopy);

	DistanceScanInfoPtr dsi = DistanceScanInfoPtr(new DistanceScanInfo);

	for (int i = 0; i < NSECTORS; i++) {
		SpicaHelper::wm->distanceScan.sectors.push_back(profile[i]);
	}

	//printf("End of FilterDistanceProfileNew.process()\n");

	return tgt;

}



void FilterDistanceProfileNew::init(){


}


void FilterDistanceProfileNew::cleanup(){

	if(profile != NULL){
		free(profile);
	}
	if(tmpProfile != NULL){
		free(tmpProfile);
	}

}

double * FilterDistanceProfileNew::getProfile(){

	return profile;

}


void FilterDistanceProfileNew::DrawCircle(int midX, int midY, int rad, unsigned char * space, int width, int height){



	int d = 3 - (2 * rad);
	int x = 0;
	int y = rad;

	space[midX * width + midY] = 128;

	while(y > x){

		if(midX + x >= 0 && midX + x < height && midY + y >= 0 && midY + y < width)
			space[(midX + x) * width + midY + y] = 128;
	
		if(midX + x >= 0 && midX + x < height && midY - y >= 0 && midY - y < width)
			space[(midX + x) * width + midY - y] = 128;
	
		if(midX - x >= 0 && midX - x < height && midY + y >= 0 && midY + y < width)
			space[(midX - x) * width + midY + y] = 128;
	
		if(midX - x >= 0 && midX - x < height && midY - y >= 0 && midY - y < width)
			space[(midX - x) * width + midY - y] = 128;
	
		if(midX + y >= 0 && midX + y < height && midY + x >= 0 && midY + x < width)
			space[(midX + y) * width + midY + x] = 128;
	
		if(midX + y >= 0 && midX + y < height && midY - x >= 0 && midY - x < width)
			space[(midX + y) * width + midY - x] = 128;
	
		if(midX - y >= 0 && midX - y < height && midY + x >= 0 && midY + x < width)
			space[(midX - y) * width + midY + x] = 128;
	
		if(midX - y >= 0 && midX - y < height && midY - x >= 0 && midY - x < width)
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


void FilterDistanceProfileNew::calculateOpponentsFromRange(Range range, double * profile, short profileLength, double sectorAngle, std::vector<Point> & opponentPoints){


		short rangeLength = range.end - range.start;
		if(rangeLength < 0)
			rangeLength += profileLength;
		rangeLength++;

		double arcusLength = rangeLength*4.0/360.0*2.0*M_PI*range.minDistance;


		double smallOpponentsValue = fabs(arcusLength/350.0 - lrint(arcusLength/350.0));
		double middleOpponentsValue = fabs(arcusLength/500.0 - lrint(arcusLength/500.0));
		double bigOpponentsValue = fabs(arcusLength/600.0 - lrint(arcusLength/600.0));

		double minValue = middleOpponentsValue;
		short numberOfOpponentsInRange = (short) lrint(arcusLength/500.0);

		if(smallOpponentsValue < middleOpponentsValue)
			numberOfOpponentsInRange = (short) lrint(arcusLength/350.0);

		if(bigOpponentsValue < smallOpponentsValue && bigOpponentsValue < middleOpponentsValue)
			numberOfOpponentsInRange = (short) lrint(arcusLength/600.0);


		//printf("NumberOfOpponentsInRange: %d\n", numberOfOpponentsInRange);

		short indexLength = rangeLength/numberOfOpponentsInRange;

		if(numberOfOpponentsInRange > 4)
			numberOfOpponentsInRange = 0;



		short currIndex = range.start;

		for(short i = 0; i < numberOfOpponentsInRange - 1; i++){

			short oldStartIndex = currIndex;
			double minDistValue = HORIZON;
			short minIndex = -1;
			for(short j = 0; j < indexLength; j++){
				if(profile[currIndex] < minDistValue){
					minDistValue = profile[currIndex];
//					minIndex =  currIndex;
				}
				if(j == indexLength/2)
					minIndex = currIndex;

				currIndex++;
				if(currIndex >= profileLength)
					currIndex = 0;

			}

			if(minDistValue < range.minDistance);
				minDistValue = range.minDistance;


			////printf("MinDistValue: %f\n", minDistValue);

			currIndex = oldStartIndex;
			for(short j = 0; j < indexLength; j++){
				////printf("CurrentIndex1: %d\n", currIndex);
				profile[currIndex] = minDistValue;
				currIndex++;
				if(currIndex >= profileLength)
					currIndex = 0;

			}

			/*currIndex++;
			if(currIndex >= profileLength)
				currIndex = 0;*/

			if(minIndex > -1){

				double oppAngle = -sectorAngle*minIndex;
				oppAngle = M_PI*oppAngle/180.0;
				if(oppAngle < -M_PI)
					oppAngle += 2.0*M_PI;


				Point opponentPoint;
				opponentPoint.x = cos(oppAngle)*(minDistValue + 250.0);
				opponentPoint.y = sin(oppAngle)*(minDistValue + 250.0);

				opponentPoints.push_back(opponentPoint);

			}


		}

		if(numberOfOpponentsInRange > 0){

			if(currIndex == range.end){
				
				if(profile[currIndex] < HORIZON){
					////printf("CurrentIndex2: %d\n", currIndex);
					double minDistValue = profile[currIndex];
					if(minDistValue < range.minDistance)
						minDistValue = range.minDistance;

					double oppAngle = -sectorAngle*currIndex;
					oppAngle = M_PI*oppAngle/180.0;
					if(oppAngle < -M_PI)
						oppAngle += 2.0*M_PI;

					////printf("MinDistValue: %f\n", minDistValue);
			
					Point opponentPoint;
					opponentPoint.x = cos(oppAngle)*(minDistValue + 250.0);
					opponentPoint.y = sin(oppAngle)*(minDistValue + 250.0);
			
					opponentPoints.push_back(opponentPoint);
				}

			}
			else {

				short oldStartIndex = currIndex;
				double minDistValue = HORIZON;
				short minIndex = -1;

				short midRange = range.end - currIndex;
				if(midRange < 0)
					midRange += profileLength;
				midRange = (midRange + 1)/2;

				short midCounter = 0;

				while(true){
					if(profile[currIndex] < minDistValue){
						minDistValue = profile[currIndex];
//						minIndex = currIndex;

					}

					if(midCounter == midRange)
						minIndex = currIndex;

					midCounter++;

					if(currIndex == range.end)
						break;
					
					currIndex++;
					if(currIndex >= profileLength)
						currIndex = 0;

				}

				currIndex = oldStartIndex;

				////printf("MinDistValue: %f\n", minDistValue);

				while(true){
					////printf("CurrentIndex3: %d\n", currIndex);
					profile[currIndex] = minDistValue;
					if(currIndex == range.end)
						break;
					
					currIndex++;
					if(currIndex >= profileLength)
						currIndex = 0;

				}

				if(minIndex > -1){
			
					double oppAngle = -sectorAngle*minIndex;
					oppAngle = M_PI*oppAngle/180.0;
					if(oppAngle < -M_PI)
						oppAngle += 2.0*M_PI;

					if(minDistValue < range.minDistance);
						minDistValue = range.minDistance;
			
					Point opponentPoint;
					opponentPoint.x = cos(oppAngle)*(minDistValue + 250.0);
					opponentPoint.y = sin(oppAngle)*(minDistValue + 250.0);
			
					opponentPoints.push_back(opponentPoint);
			
				}


			}
		}


}


