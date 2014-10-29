/*
 * $Id: FilterLinePoints.cpp 1935 2007-03-19 19:50:12Z phbaer $
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
#include "FilterLinePoints.h"

#include <algorithm>
#include <math.h>
#include <boost/shared_ptr.hpp>

#define BLOB_UNDEF 100000

FilterLinePoints::FilterLinePoints(int area):Filter(OF_ZERO, area, area){

	this->sc = SystemConfig::getInstance();

	MX = area/2;
	MY = area/2;

	Configuration *loc = (*this->sc)["Localization"];

	LinePointsThreshold = (unsigned char)loc->get<int>("Localization", "LinePointsThreshold", NULL);
	LinePointsJump = (unsigned char)loc->get<int>("Localization", "LinePointsJump", NULL);
	MinLineWidth = (unsigned char)loc->get<int>("Localization", "MinLineWidth", NULL);
	MaxLineWidth = (unsigned char)loc->get<int>("Localization", "MaxLineWidth", NULL);
	OnlyFirstPoint = loc->get<bool>("Localization", "OnlyFirstPoint", NULL);

	Configuration *vision = (*this->sc)["Vision"];

	negRanges[0][0] = vision->get<short>("Vision", "Holder", "NegRange_0_0", NULL);
	negRanges[0][1] = vision->get<short>("Vision", "Holder", "NegRange_0_1", NULL);
	negRanges[1][0] = vision->get<short>("Vision", "Holder", "NegRange_1_0", NULL);
	negRanges[1][1] = vision->get<short>("Vision", "Holder", "NegRange_1_1", NULL);
	negRanges[2][0] = vision->get<short>("Vision", "Holder", "NegRange_2_0", NULL);
	negRanges[2][1] = vision->get<short>("Vision", "Holder", "NegRange_2_1", NULL);

	short nLines = vision->get<short>("Vision", "NumberScanLines", NULL);
	
	boost::shared_ptr<std::vector<std::string> > holdersBPtr = (*vision).getSections("Vision", "Holder", NULL);
	std::vector<std::string> * holders = holdersBPtr.get();

	addHolders.clear();

	for(unsigned int i = 0; i < holders->size(); i++){

		Holder addHolder;
		addHolder.start = vision->get<double>("Vision", "Holder", (*holders)[i].c_str(), "Start", NULL);
		addHolder.end = vision->get<double>("Vision", "Holder", (*holders)[i].c_str(), "End", NULL);
		addHolders.push_back(addHolder);

		printf("Holder: %s %f %f\n", (*holders)[i].c_str(), addHolder.start, addHolder.end);

	}

	angleValidity = (char *) malloc(360);
	for(unsigned int j = 0; j < 360; j++){

		bool validity = true;

		short i = (short) lrint(j/360.0*nLines);
		if(i >= 360)
			i = 0;

		if(negRanges[0][0] > nLines/2 && negRanges[0][1] < nLines/2){

			if( (i >= negRanges[0][0]) || (i <= negRanges[0][1]) || 
				(i >= negRanges[1][0] && i <= negRanges[1][1]) ||
				(i >= negRanges[2][0] && i <= negRanges[2][1]))
				validity = false;

		}
		else {

			if( (i >= negRanges[0][0] && i <= negRanges[0][1]) || 
				(i >= negRanges[1][0] && i <= negRanges[1][1]) ||
				(i >= negRanges[2][0] && i <= negRanges[2][1]))
				validity = false;

		}

		double lineAngle = 1.0*j;

		for(unsigned a = 0; a < addHolders.size(); a++){
			if(addHolders[a].start > 180.0 && addHolders[a].end < 180.0){

				if(lineAngle >= addHolders[a].start || lineAngle <= addHolders[a].end)
					validity = false;

			}
			else {

				if(lineAngle >= addHolders[a].start && lineAngle <= addHolders[a].end)
					validity = false;

			}

		}

		angleValidity[j] = validity?1:0;

	}


	init();

}


FilterLinePoints::FilterLinePoints(int width, int height):Filter(OF_ZERO, width, height){

	this->sc = SystemConfig::getInstance();

	Configuration *vision = (*this->sc)["Vision"];
	Configuration *loc = (*this->sc)["Localization"];

	MX = vision->get<int>("Vision", "CameraMX", NULL);
	MY = vision->get<int>("Vision", "CameraMY", NULL);

	LinePointsThreshold = (unsigned char)loc->get<int>("Localization", "LinePointsThreshold", NULL);
	LinePointsJump = (unsigned char)loc->get<int>("Localization", "LinePointsJump", NULL);
	MinLineWidth = (unsigned char)loc->get<int>("Localization", "MinLineWidth", NULL);
	MaxLineWidth = (unsigned char)loc->get<int>("Localization", "MaxLineWidth", NULL);
	OnlyFirstPoint = loc->get<bool>("Localization", "OnlyFirstPoint", NULL);

	//Configuration *vision = (*this->sc)["Vision"];

	negRanges[0][0] = vision->get<short>("Vision", "Holder", "NegRange_0_0", NULL);
	negRanges[0][1] = vision->get<short>("Vision", "Holder", "NegRange_0_1", NULL);
	negRanges[1][0] = vision->get<short>("Vision", "Holder", "NegRange_1_0", NULL);
	negRanges[1][1] = vision->get<short>("Vision", "Holder", "NegRange_1_1", NULL);
	negRanges[2][0] = vision->get<short>("Vision", "Holder", "NegRange_2_0", NULL);
	negRanges[2][1] = vision->get<short>("Vision", "Holder", "NegRange_2_1", NULL);

	short nLines = vision->get<short>("Vision", "NumberScanLines", NULL);
	

	boost::shared_ptr<std::vector<std::string> > holdersBPtr = (*vision).getSections("Vision", "Holder", NULL);
	std::vector<std::string> * holders = holdersBPtr.get();

	addHolders.clear();

	for(unsigned int i = 0; i < holders->size(); i++){

		Holder addHolder;
		addHolder.start = vision->get<double>("Vision", "Holder", (*holders)[i].c_str(), "Start", NULL);
		addHolder.end = vision->get<double>("Vision", "Holder", (*holders)[i].c_str(), "End", NULL);
		addHolders.push_back(addHolder);

		printf("Holder: %s %f %f\n", (*holders)[i].c_str(), addHolder.start, addHolder.end);

	}

	angleValidity = (char *) malloc(360);
	for(unsigned int j = 0; j < 360; j++){

		bool validity = true;

		short i = (short) lrint(j/360.0*nLines);
		if(i >= 360)
			i = 0;

		if(negRanges[0][0] > nLines/2 && negRanges[0][1] < nLines/2){

			if( (i >= negRanges[0][0]) || (i <= negRanges[0][1]) || 
				(i >= negRanges[1][0] && i <= negRanges[1][1]) ||
				(i >= negRanges[2][0] && i <= negRanges[2][1]))
				validity = false;

		}
		else {

			if( (i >= negRanges[0][0] && i <= negRanges[0][1]) || 
				(i >= negRanges[1][0] && i <= negRanges[1][1]) ||
				(i >= negRanges[2][0] && i <= negRanges[2][1]))
				validity = false;

		}

		double lineAngle = 1.0*j;

		for(unsigned a = 0; a < addHolders.size(); a++){
			if(addHolders[a].start > 180.0 && addHolders[a].end < 180.0){

				if(lineAngle >= addHolders[a].start || lineAngle <= addHolders[a].end)
					validity = false;

			}
			else {

				if(lineAngle >= addHolders[a].start && lineAngle <= addHolders[a].end)
					validity = false;

			}

		}

		angleValidity[j] = validity?1:0;

	}

	init();

}



FilterLinePoints::~FilterLinePoints(){

	cleanup();

}
		

unsigned char * FilterLinePoints::process(unsigned char * src, unsigned int width, unsigned int height, std::vector<LinePoint> & LinePoints, DistanceLookupHelper & distanceHelper, ScanLineHelper & helper){

	unsigned char * tgt = src;

	double * LookupTable = distanceHelper.getLookupTable();

	short * firstInner = helper.getInnerLines();
	short * nInner = helper.getInnerLinesN();

	short * firstOuter = helper.getOuterLines();
	short * nOuter = helper.getOuterLinesN();

	short maxPoints = helper.getMaxPoints();

	short x;
	short y;
	int floorBrightness = 240;

/////////////////
/*
	int linePointInd[5000];
	int lpCount=0;
*/
/////////////////

	LinePoints.clear();
	std::vector<short> LinePointsX;
	LinePointsX.clear();
	std::vector<short> LinePointsY;
	LinePointsY.clear();


	for(short i = 0; i < helper.getNumberLines(); i++){

		short b = 0;
		short e = 0;


		if(i % 2 == 0){

			short * line = firstInner;

			x = *line++;
			y = *line++;

			short vb = src[x*width + y];

			for(short j = 1; j < (*nInner); j++){
				x = *line++;
				y = *line++;
				short va = src[x*width + y];

				if(va > vb + LinePointsJump && va > LinePointsThreshold){
					b = j;
					e = j;
				}

				if(va < vb - LinePointsJump && va < floorBrightness){
					e = j;
				}

				if(b > 0 & e-b > MinLineWidth & e-b < MaxLineWidth + 10){

					short indX = firstInner[((e+b)/2)*2];
					short indY = firstInner[((e+b)/2)*2 + 1];

					double angle = -atan2(1.0*indY - MY, 1.0*indX - MX);
					double dist = LookupTable[indX*width + indY];



					if(dist > 100.0 && dist < 7500.0 && (double) (e-b) < (double) (2*MaxLineWidth/(dist/1000.0))){

						//tgt[indX*width + indY] = 0;
						//linePointInd[lpCount++] = indX*width + indY;

						LinePoint p;
						p.x = dist*cos(angle);
						p.y = dist*sin(angle);
						angle = -angle;
						if(angle < 0.0)
							angle += 2.0*M_PI;

						short angleInd = (short) lrint(angle*360.0/(2.0*M_PI));
						if(angleInd >= 360)
							angleInd = 0;
						//printf("AngleInd: %d AngleValidity %d\n", angleInd, angleValidity[angleInd]);					
						if(angleValidity[angleInd]){
							LinePoints.push_back(p);
							LinePointsX.push_back(indX);
							LinePointsY.push_back(indY);
						}
						if(OnlyFirstPoint)
							break;
					}

					//printf("LinePoint: %d %d %d\n", i, b, e);
					//printf("LinePointAngle: %f\n", angle/M_PI * 180);
					//printf("LinePointIndex: %d %d\n", indX - height/2, indY - width/2);

					b = 0;
					e = 0;



				}
				vb = va;
			}

			firstInner = firstInner + maxPoints*2;
			nInner++;
	

		}
		else {

			short * line = firstOuter;
			x = *line++;
			y = *line++;

			short vb = src[x*width + y];

			for(short j = 1; j < (*nOuter); j++){
				x = *line++;
				y = *line++;
				short va = src[x*width + y];

				if(va > vb + LinePointsJump && va > LinePointsThreshold){
					b = j;
					e = j;
				}

				if(va < vb - LinePointsJump && va < floorBrightness){
					e = j;
				}

				if(b > 0 & e-b > MinLineWidth & e-b < MaxLineWidth + 5){

					short indX = firstOuter[((e+b)/2)*2];
					short indY = firstOuter[((e+b)/2)*2 + 1];

					double angle = -atan2(1.0*indY - MY, 1.0*indX - MX);
					double dist = LookupTable[indX*width + indY];
					


					if(dist > 100.0 && dist < 6500.0 && (double) (e-b) < (double) (2*MaxLineWidth/(dist/1000.0))){

						//tgt[indX*width + indY] = 0;
						//linePointInd[lpCount++] = indX*width + indY;

						LinePoint p;
						p.x = dist*cos(angle);
						p.y = dist*sin(angle);
						angle = -angle;
						if(angle < 0.0)
							angle += 2.0*M_PI;
						
						short angleInd = (short) lrint(angle*360.0/(2.0*M_PI));
						if(angleInd >= 360)
							angleInd = 0;
						
						if(angleValidity[angleInd]){
							LinePoints.push_back(p);
							LinePointsX.push_back(indX);
							LinePointsY.push_back(indY);
						}
						if(OnlyFirstPoint)
							break;
					}


					//printf("LinePoint: %d %d %d\n", i, b, e);
					//printf("LinePointAngle: %f\n", angle/M_PI * 180);
					//printf("LinePointIndex: %d %d\n", indX - height/2, indY - width/2);

					b = 0;
					e = 0;

					

				}
				vb = va;
			}

			firstOuter = firstOuter + maxPoints;
			nOuter++;

		}




	}

	printf("FilterLinePoints - Number of LinePoints(Circle): %d\n", (int)LinePoints.size());

	short numberOfCircles = helper.getNumberCircles();
	short * circles = helper.getCircles();
	short * circleOffsets = helper.getCircleOffsets();

	for(short a = 0; a < numberOfCircles; a++){

		short * circlePtr = circles + 2*circleOffsets[a];
		short * circlePtrBegin = circles + 2*circleOffsets[a];
		x = *circlePtr++;
		y = *circlePtr++;

		short vb = src[x*width + y];

		short b = 0;
		short e = 0;

		short j = 1;

		for(short i = circleOffsets[a] + 1; i < circleOffsets[a+1]; i++){

			x = *circlePtr++;
			y = *circlePtr++;
			short va = src[x*width + y];

			if(va > vb + LinePointsJump && va > LinePointsThreshold){
				b = j;
				e = j;
			}

			if(va < vb - LinePointsJump && va < floorBrightness){
				e = j;
			}

			if(b > 0 & e-b >= 3 & e-b < MaxLineWidth + 5){

				short indX = circlePtrBegin[((e+b)/2)*2];
				short indY = circlePtrBegin[((e+b)/2)*2 + 1];

				double angle = -atan2(1.0*indY - MY, 1.0*indX - MX);
				double dist = LookupTable[indX*width + indY];
					


				if(dist > 100.0 && dist < 6000.0 && (double) (e-b) < (double) (3.0*MaxLineWidth/(dist/1000.0))){

					

					LinePoint p;
					p.x = dist*cos(angle);
					p.y = dist*sin(angle);
					angle = -angle;	
					if(angle < 0.0)
						angle += 2.0*M_PI;
					
					short angleInd = (short) lrint(angle*360.0/(2.0*M_PI));
					if(angleInd >= 360)
						angleInd = 0;
					
					if(angleValidity[angleInd]){
						LinePoints.push_back(p);
						tgt[indX*width + indY] = 0;
					}

					b = 0;
					e = 0;

					

				}
				
			}

			vb = va;
			j++;
		}

	}


	for(unsigned int i = 0; i < LinePointsX.size(); i++){
		tgt[LinePointsX[i]*width + LinePointsY[i]] = 0;

	}


	printf("FilterLinePoints - Number of LinePoints: %d\n", (int)LinePoints.size());

/////////////
/*
	for(int i=0; i<lpCount; i++) {
		tgt[linePointInd[i]]=0;
	}
*/
/////////////

	return tgt;

}


void FilterLinePoints::init(){


}


void FilterLinePoints::cleanup(){


}



