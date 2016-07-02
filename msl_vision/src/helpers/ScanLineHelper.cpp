/*
 * $Id: ScanLineHelper.cpp 1987 2007-04-09 16:58:10Z rreichle $
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
#include "ScanLineHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>


ScanLineHelper::ScanLineHelper() : sc() {

	this->sc = SystemConfig::getInstance();

	Configuration *vision = (*this->sc)["Vision"];

	//mx = short(atoi(vision->Values["CameraMX"].c_str()));
	//my = short(atoi(vision->Values["CameraMY"].c_str()));
	iRadius = vision->get<short>("Vision", "ScanLinesInnerRadius", NULL);
	oRadius = vision->get<short>("Vision", "ScanLinesOuterRadius", NULL);
	nLines = vision->get<short>("Vision", "NumberScanLines", NULL);
	frontScanlineDistance = vision->get<short>("Vision", "FrontScanlineDistance", NULL);

	int imWidth = vision->get<int>("Vision", "ImageArea", NULL);
	int imHeight = vision->get<int>("Vision", "ImageArea", NULL);

	printf("ScanLineHelper MX: %d\n", imWidth/2);
	printf("ScanLineHelper MY: %d\n", imHeight/2);

	maxPoints = MAXPOINTS;

	mx = (short)imHeight/2;
	my = (short)imWidth/2;

	scWIDTH = imWidth;
	scHEIGHT = imHeight;

	init();


}




ScanLineHelper::ScanLineHelper(int imWidth, int imHeight) : sc() {

	Configuration *vision = (*this->sc)["Vision"];

	mx = vision->get<short>("Vision", "CameraMX", NULL);
	my = vision->get<short>("Vision", "CameraMY", NULL);

	iRadius = vision->get<short>("Vision", "ScanLinesInnerRadius", NULL);
	oRadius = vision->get<short>("Vision", "ScanLinesOuterRadius", NULL);
	nLines = vision->get<short>("Vision", "NumberScanLines", NULL);
	frontScanlineDistance = vision->get<short>("Vision", "FrontScanlineDistance", NULL);

	printf("ScanLineHelper MX: %d\n", mx);
	printf("ScanLineHelper MY: %d\n", my);

	maxPoints = MAXPOINTS;

	//mx = (short)imHeight/2;
	//my = (short)imWidth/2;

	scWIDTH = imWidth;
	scHEIGHT = imHeight;

	init();


}


ScanLineHelper::~ScanLineHelper(){

	printf("Destructor of ScanLineHelper\n");

	free(innerLines);
	free(innerLinesN);
	free(outerLines);
	free(outerLinesN);
	free(circles);



}



void ScanLineHelper::init( ){

	short ax = 0;
	short ay = 0;
	short ex = 0;
	short ey = 0;

	numberOfCircles = 29;

	innerLines = (short *) malloc(nLines/2 * maxPoints * 2 * sizeof(short));
	innerLinesN = (short *) malloc(nLines/2 * sizeof(short));
	outerLines = (short *) malloc(nLines/2 * maxPoints/2 * 2 * sizeof(short));
	outerLinesN = (short *) malloc(nLines/2 * sizeof(short));
	circles = (short *) malloc(1500*numberOfCircles*2*sizeof(short));
	circleOffsets = (short *) malloc((numberOfCircles + 1)*sizeof(short));

	printf("Sizeof short: %d\n", (int)sizeof(short));

	for(int i = 0; i < nLines; i++){

		double angle = i*1.0/nLines *2*M_PI;


		if(i % 2 == 0){
			if(fabs(angle) < 4*M_PI/5 || fabs(angle) > 6*M_PI/5) {
				ax = (short) (cos(angle)*iRadius + mx);
				ay = (short) (sin(angle)*iRadius + my);
			} else {
				double helpRadius = -frontScanlineDistance / cos(angle);
				ax = (short) (cos(angle)*helpRadius + mx);
				ay = (short) (sin(angle)*helpRadius + my);
			}

			ex = (short) (cos(angle)*(double) oRadius + (double) mx);
			ey = (short) (sin(angle)*(double) oRadius + (double) my);

			int offset = (i/2)*maxPoints*2;
			DrawLine(innerLines + offset, innerLinesN + (i/2), ax, ay, ex, ey);

		}
		else{

			ax = (short) (cos(angle)*(iRadius + (oRadius - iRadius)/4) + mx);
			ay = (short) (sin(angle)*(iRadius + (oRadius - iRadius)/4) + my);

			ex = (short) (cos(angle)*oRadius + mx);
			ey = (short) (sin(angle)*oRadius + my);
			
			int offset = (i/2)*maxPoints;
			DrawLine(outerLines + offset, outerLinesN + (i/2), ax, ay, ex, ey);
		}



	}

	initCircles();

}

void ScanLineHelper::DrawLine(short * line, short * nPoints, short ax, short ay, short ex, short ey){

	short x = ax;
	short y = ay;
	short D = 0;
	short HX = ex - ax;
	short HY = ey - ay;
	short xInc = 1;
	short yInc = 1;

	//printf("ax: %d ay: %d ex: %d ey: %d\n", ax, ay, ex, ey);

	//printf("linePointer: %d\n", line);
	//printf("nPointer: %d\n", nPoints);
		
	if(HX < 0) {
		xInc = -1; 
		HX = -HX;
	}

	if(HY < 0) {
		yInc = -1; 
		HY = -HY;
	}

	if(HY <= HX) {
		short c = 2 * HX; 
		short M = 2 * HY;
		short counter = 0;
		while(1) { 

			if(x >= 0 && x < scHEIGHT && y >= 0 && y < scWIDTH){
		
				*line++ = x;
				*line++ = y;
				counter = counter + 1;
			}
			else{
				//printf("out of image!!!!!!!!!!!!!!!!!!!!!!!!!!!!1\n");
				break;
			}


			if(x == ex) 
				break;

			x = x + xInc;
			D = D + M;

			if(D > HX) {
				y = y + yInc; 
				D = D - c;
			}
		}
		*nPoints = counter;
	}	
	else {

		short c = 2 * HY; 
		short M = 2 * HX;
		short counter = 0;
		while(1) {
			
			if(x >= 0 && x < scHEIGHT && y >= 0 && y < scWIDTH){

				*line++ = x;
				*line++ = y;

				counter = counter + 1;

			}
			else{
				//printf("out of image!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
				break;
			}

			if(y == ey) 
				break;

			y = y + yInc;
			D = D + M;

			if(D > HY) {
				x = x + xInc; 
				D = D - c;
			}
		}
		*nPoints = counter;
	}

	//printf("Size of line: %d\n", (*nPoints));

}


void ScanLineHelper::initCircles( ){

	short currOffset = 0;

	short * circlePtr = circles;

	for(short a = 0; a < numberOfCircles; a++){

		circleOffsets[a] = currOffset;
		
		short radius = 60 + a*6;

		short currX = 0;
		short currY = 0;

		currX = mx + radius;
		currY = my;

		//segment 1
		while(currX >= mx && currY >= my - radius){

			double m = 2.0*radius*radius;
			short indX = -1;
			short indY = -1;
			short cX = 0;
			short cY = 0;

			for(short i = -1; i <= 0; i++){
				for(short j = -1; j <= 0; j++){

					if(i != 0 || j != 0){
					
						cX = currX + i;
						cY = currY + j;

						double f = fabs((cX - mx)*(cX - mx) + (cY - my)*(cY - my) - radius*radius*1.0);
		
						if(f < m){
							m = f;
							indX = cX;
							indY = cY;
						}

					}
				}
			}

			currX = indX;
			currY = indY;

			if(currX < 0 || currX >= scHEIGHT || currY < 0 || currY >= scWIDTH){
				//printf("Circles out of bounds!\n");
			}

			*circlePtr++ = currX;
			*circlePtr++ = currY;
			currOffset++;

			//printf("Circles currX : %d currY : %d\n", currX, currY);

		}

		//segment 2
		while(currX >= mx - radius && currY <= my){

			double m = 2.0*radius*radius;
			short indX = -1;
			short indY = -1;
			short cX = 0;
			short cY = 0;

			for(short i = -1; i <= 0; i++){
				for(short j = 0; j <= 1; j++){

					if(i != 0 || j != 0){
					
						cX = currX + i;
						cY = currY + j;

						double f = fabs((cX - mx)*(cX - mx) + (cY - my)*(cY - my) - radius*radius*1.0);
		
						if(f < m){
							m = f;
							indX = cX;
							indY = cY;
						}

					}
				}
			}

			currX = indX;
			currY = indY;

			*circlePtr++ = currX;
			*circlePtr++ = currY;
			currOffset++;
		}



		while(currX <= mx && currY <= my + radius){

			double m = 2.0*radius*radius;
			short indX = -1;
			short indY = -1;
			short cX = 0;
			short cY = 0;

			for(short i = 0; i <= 1; i++){
				for(short j = 0; j <= 1; j++){

					if(i != 0 || j != 0){
					
						cX = currX + i;
						cY = currY + j;

						double f = fabs((cX - mx)*(cX - mx) + (cY - my)*(cY - my) - radius*radius*1.0);
		
						if(f < m){
							m = f;
							indX = cX;
							indY = cY;
						}

					}
				}
			}

			currX = indX;
			currY = indY;

			*circlePtr++ = currX;
			*circlePtr++ = currY;
			currOffset++;
		}



		while(currX <= mx + radius && currY >= my){

			double m = 2.0*radius*radius;
			short indX = -1;
			short indY = -1;
			short cX = 0;
			short cY = 0;

			for(short i = 0; i <= 1; i++){
				for(short j = -1; j <= 0; j++){

					if(i != 0 || j != 0){
					
						cX = currX + i;
						cY = currY + j;

						double f = fabs((cX - mx)*(cX - mx) + (cY - my)*(cY - my) - radius*radius*1.0);
		
						if(f < m){
							m = f;
							indX = cX;
							indY = cY;
						}

					}
				}
			}

			currX = indX;
			currY = indY;

			*circlePtr++ = currX;
			*circlePtr++ = currY;
			currOffset++;
		}

	}

	printf("Circles currOffset = %d\n", currOffset);
	circleOffsets[numberOfCircles] = currOffset;

}


short * ScanLineHelper::getInnerLines(){

	return innerLines;

}


short * ScanLineHelper::getInnerLinesN(){

	return innerLinesN;

}


short * ScanLineHelper::getOuterLines(){

	return outerLines;

}


short * ScanLineHelper::getOuterLinesN(){

	return outerLinesN;

}


short ScanLineHelper::getNumberLines(){

	return nLines;

}


short ScanLineHelper::getMaxPoints(){

	return maxPoints;

}

short * ScanLineHelper::getCircles(){

	return circles;

}

short * ScanLineHelper::getCircleOffsets(){

	return circleOffsets;

}


short ScanLineHelper::getNumberCircles(){

	return numberOfCircles;

}
