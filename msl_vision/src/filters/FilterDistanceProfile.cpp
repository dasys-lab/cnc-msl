/*
 * $Id: FilterDistanceProfile.cpp 1935 2007-03-19 19:50:12Z phbaer $
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
#include "FilterDistanceProfile.h"

#include <stdio.h>
#include <stdlib.h>
#include <msl_sensor_msgs/DistanceScanInfo.h>
#include <math.h>

#include "../helpers/Lookuptable.h"
#include "../helpers/SpicaHelper.h"

#define CALIB_MAX 60

FilterDistanceProfile::FilterDistanceProfile(int width, int height, bool _calibMode):Filter(OF_ZERO, width, height), sc() {

	this->sc = SystemConfig::getInstance();

	profile = NULL;
	tmpProfile = NULL;

	calibProfile = NULL;
	calibMode = _calibMode;
	calibCounter = 0;

	//std::string config_file = "EndPoints.conf";
	//ConfigHelper configHelper(config_file);

	//ConfigElement * modules = configHelper.rootElement->childrenMap["EndPoints"];
	//ConfigElement * worldModel = modules->childrenMap["WorldModel"];

	//std::string socketSpec = worldModel->Values["Listen"];

	//ConfigHelper::parseSocketSpec(socketSpec, socketType, destAddress, destPort);
	//printf("destPort from DistanceProfile: %d\n", destPort);

	//if(socketType == "udp")
	//	socket = new Anja::DatagramSocket();
	//else {
	//	socket = new Anja::UnixSocket();
	//	((Anja::UnixSocket*) socket)->connect(destAddress);
	//}

	Configuration *vision = (*this->sc)["Vision"];

	negRanges[0][0] = vision->get<short>("Vision", "Holder", "NegRange_0_0", NULL);
	negRanges[0][1] = vision->get<short>("Vision", "Holder", "NegRange_0_1", NULL);
	negRanges[1][0] = vision->get<short>("Vision", "Holder", "NegRange_1_0", NULL);
	negRanges[1][1] = vision->get<short>("Vision", "Holder", "NegRange_1_1", NULL);
	negRanges[2][0] = vision->get<short>("Vision", "Holder", "NegRange_2_0", NULL);
	negRanges[2][1] = vision->get<short>("Vision", "Holder", "NegRange_2_1", NULL);

	numberOfLines = -1;

	init();

}



FilterDistanceProfile::~FilterDistanceProfile(){

	cleanup();

}
		

unsigned char * FilterDistanceProfile::process(unsigned char * src, unsigned int width, unsigned int height, unsigned char color, ScanLineHelper & helper, DistanceLookupHelper & distanceHelper, bool printOutput){

	unsigned char * tgt = src;

	numberOfLines = helper.getNumberLines();

	double * LookupTable = distanceHelper.getLookupTable();

	if(profile == NULL){
		profile = (double *) malloc(NSECTORS*sizeof(double));

	}
	if(tmpProfile == NULL){
		tmpProfile = (double *) malloc(helper.getNumberLines()*sizeof(double));

	}

	if(calibMode && calibProfile == NULL){
		
		calibProfile = (int *) malloc(helper.getNumberLines()*sizeof(int));
		bzero(calibProfile, helper.getNumberLines()*sizeof(int));

	}

	for(int i = 0; i < NSECTORS; i++){

		tmpProfile[i] = HORIZON;
	}

	short * firstInner = helper.getInnerLines();
	short * nInner = helper.getInnerLinesN();

	short * firstOuter = helper.getOuterLines();
	short * nOuter = helper.getOuterLinesN();

	//printf("Number of ScanLines: %d\n", helper.getNumberLines());

	short maxPoints = helper.getMaxPoints();

	short x;
	short y;
	short jumpx;
	short jumpy;

	for(short i = 0; i < helper.getNumberLines(); i++){

		bool found = false;
		short counter = 0;
		short indX = 0;
		short indY = 0;

//		short redAreaBegin = -1;
//		short redAreaEnd = -1;

		if(i % 2 == 0){

			short * line = firstInner;

			for(short j = 0; j < (*nInner); j++){

				short jump = (short) rint(30.0 - j/5.0);
				if(jump < 3)
					jump = 3;
				if(j > 150)
					jump = 0;

				jumpx = line[jump*2];
				jumpy = line[jump*2 + 1];

				x = *line++;
				y = *line++;

				if(src[x*width + y] == color && src[jumpx*width + jumpy] != COLOR_RED){
					counter++;

					if(counter == 1){
						indX = x;
						indY = y;
					}
					if(counter >= RANGE){
						found = true;
						break;
					}
				}
				else{
					counter = 0;
				}
			}

			firstInner = firstInner + maxPoints*2;
			nInner++;
	
		}
		else {

			short * line = firstOuter;

			for(short j = 0; j < (*nOuter); j++){

				short jump = (short) rint(15.0 - j/5.0);
				if(jump < 3)
					jump = 3;
				if(j > 150)
					jump = 0;

				jumpx = line[jump*2];
				jumpy = line[jump*2 + 1];

				x = *line++;
				y = *line++;


				if(src[x*width + y] == color && src[jumpx*width + jumpy] != COLOR_RED){
					counter++;

					if(counter == 1){
						indX = x;
						indY = y;
					}

					if(counter >= RANGE){
						found = true;
						break;
					}
				}
				else{
					counter = 0;

				}
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

			if(calibProfile != NULL && calibCounter >= 10 && calibCounter <= CALIB_MAX && tmpProfile[i] < 4000.0){
				calibProfile[i]++;
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

		}
		else{
			tmpProfile[i] = HORIZON;

		}

	}

/*
	if(tmpProfile[negRanges[0][0] - 2] < HORIZON || tmpProfile[negRanges[0][1] + 2] < HORIZON){
		double minDist = tmpProfile[negRanges[0][0] - 2];
		if(minDist < tmpProfile[negRanges[0][1] + 2])
			minDist = tmpProfile[negRanges[0][1] + 2];

		for(short i = negRanges[0][0] - 1; i < helper.getNumberLines(); i++){
			tmpProfile[i] = minDist;
		}

		for(short i = 0; i <= negRanges[0][1] + 1; i++){
			tmpProfile[i] = minDist;
		}


	}

	if(tmpProfile[negRanges[1][0] - 1] < HORIZON || tmpProfile[negRanges[1][1] + 1] < HORIZON){
		double minDist = tmpProfile[negRanges[1][0] - 1];
		if(minDist < tmpProfile[negRanges[1][1] + 1])
			minDist = tmpProfile[negRanges[1][1] + 1];

		for(short i = negRanges[1][0]; i <= negRanges[1][1]; i++){
			tmpProfile[i] = minDist;
		}

	}

	if(tmpProfile[negRanges[2][0] - 1] < HORIZON || tmpProfile[negRanges[2][1] + 1] < HORIZON){
		double minDist = tmpProfile[negRanges[2][0] - 1];
		if(minDist < tmpProfile[negRanges[2][1] + 1])
			minDist = tmpProfile[negRanges[2][1] + 1];

		for(short i = negRanges[2][0]; i <= negRanges[2][1]; i++){
			tmpProfile[i] = minDist;
		}

	}*/

	double currentMin = tmpProfile[0];
	short index = helper.getNumberLines() - 1;
	short sector = 0;
	short nLines = helper.getNumberLines()/NSECTORS;
	short counter = 1;


	while(index >= 0){
		
		if(counter == nLines){
			if(currentMin > tmpProfile[index])
				currentMin = tmpProfile[index];
			profile[sector] = currentMin;
			sector++;
			currentMin = tmpProfile[index];
			counter = 1;
		}
		else{
			if(currentMin > tmpProfile[index])
				currentMin = tmpProfile[index];
			counter++;	
		}
		index--;
	}


	if(false && printOutput){
		for(int i = 0; i < helper.getNumberLines(); i++){

			if(tmpProfile[i] > 6000.0)
				printf("DP: %d %f\n", i, 0.0);
			else
				printf("DP: %d %f\n", i, tmpProfile[i]);

		}

	}

	SpicaHelper::wm->distanceScan.sectors.clear();
	for (int i = 0; i < NSECTORS; i++) {
		SpicaHelper::wm->distanceScan.sectors.push_back(profile[i]);
	}


	calibCounter++;

	return tgt;

}



void FilterDistanceProfile::init(){


}


void FilterDistanceProfile::cleanup(){

	if(profile != NULL){
		free(profile);
	}
	if(tmpProfile != NULL){
		free(tmpProfile);
	}
	if(calibProfile != NULL){
		free(calibProfile);
	}
//	delete socket;

}

double * FilterDistanceProfile::getProfile(){

	return profile;

}

short * FilterDistanceProfile::calculateNewNegRanges(){

	if(calibMode && calibCounter > CALIB_MAX){		

		int ind1 = 170;
		int ind2 = numberOfLines/3 - 10;
		int ind3 = 2*numberOfLines/3 - 10;


		bool found1 = false;
		bool found2 = false;
		bool found3 = false;

		negRanges[0][0] = -1;
		negRanges[0][1] = -1;
		negRanges[1][0] = -1;
		negRanges[1][1] = -1;
		negRanges[2][0] = -1;
		negRanges[2][1] = -1;

	
		for(int i = 0; i < 20; i++){

			ind1 = ind1 % numberOfLines;
			if(calibProfile[ind1] > 3){
				if(found1)
					negRanges[0][1] = ind1;
				else {
					negRanges[0][0] = ind1;
					negRanges[0][1] = ind1;
					found1 = true;
				}
			}	

			ind2 = ind2 % numberOfLines;
			if(calibProfile[ind2] > 3){
				if(found2)
					negRanges[1][1] = ind2;
				else {
					negRanges[1][0] = ind2;
					negRanges[1][1] = ind2;
					found2 = true;
				}
			}	

			ind3 = ind3 % numberOfLines;
			if(calibProfile[ind3] > 3){
				if(found3)
					negRanges[2][1] = ind3;
				else {
					negRanges[2][0] = ind3;
					negRanges[2][1] = ind3;
					found3 = true;
				}
			}

			ind1++;
			ind2++;
			ind3++;	

			
		} 

	}

	return ((short*) negRanges);

}



