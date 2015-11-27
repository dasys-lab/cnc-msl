/*
 * $Id: ScanLineHelperBall.cpp 1987 2007-04-09 16:58:10Z rreichle $
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
#include "ScanLineHelperDirected.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>


ScanLineHelperDirected::ScanLineHelperDirected() : sc() {

	printf("ScanLineHelper Ball Constructor\n");

	this->sc = SystemConfig::getInstance();

	Configuration *vision = (*this->sc)["Vision"];

	//mx = short(atoi(vision->Values["CameraMX"].c_str()));
	//my = short(atoi(vision->Values["CameraMY"].c_str()));
	iRadius = vision->get<short>("Vision", "ScanLinesInnerRadiusBall", NULL);
	oRadius = vision->get<short>("Vision", "ScanLinesOuterRadius", NULL);
	nLines = vision->get<short>("Vision", "NumberScanLinesBall", NULL);

	int imWidth = vision->get<int>("Vision", "ImageArea", NULL);
	int imHeight = vision->get<int>("Vision", "ImageArea", NULL);

	printf("ScanLineHelperBall MX: %d\n", imWidth/2);
	printf("ScanLineHelperBall MY: %d\n", imHeight/2);

	maxPoints = MAXPOINTSDIRECTED;

	mx = (short)imHeight-1;
	my = (short)imWidth/2;

	scWIDTH = imWidth;
	scHEIGHT = imHeight;

	init();


}




ScanLineHelperDirected::ScanLineHelperDirected(int imWidth, int imHeight) : sc() {

	printf("ScanLineHelper Ball Constructor\n");

	this->sc = SystemConfig::getInstance();
	Configuration *vision = (*this->sc)["Vision"];

	//mx = vision->get<short>("Vision", "CameraMX", NULL);
	//my = vision->get<short>("Vision", "CameraMY", NULL);

	iRadius = vision->get<short>("Vision", "ScanLinesInnerRadiusDirected", NULL);
	oRadius = vision->get<short>("Vision", "ScanLinesOuterRadiusDirected", NULL);
	nLines = vision->get<short>("Vision", "NumberScanLinesDirected", NULL);


	maxPoints = MAXPOINTSDIRECTED;

	mx = (short)imHeight/2;
	my = (short)1;

	printf("ScanLineHelperDirected MX: %d\n", mx);
	printf("ScanLineHelperDirected MY: %d\n", my);

	scWIDTH = imWidth;
	scHEIGHT = imHeight;

	init();


}


ScanLineHelperDirected::~ScanLineHelperDirected(){

	printf("Destructor of ScanLineHelperBall\n");

	free(lines);
	free(lineOffsets);

}



void ScanLineHelperDirected::init(){

	printf("ScanLineHelper Ball Init\n");

	short ax = 0;
	short ay = 0;
	short ex = 0;
	short ey = 0;

	lines = (short *) malloc(nLines * maxPoints * 2 * sizeof(short));
	lineOffsets = (int *) malloc((nLines + 1) * sizeof(int));

	int offsetCounter = 0;


	printf("Sizeof short: %d\n", (int)sizeof(short));

	double angleMin = 0.2*M_PI/2.0;
	double angleMax = 1.8*M_PI/2.0;

	for(int i = 0; i < nLines; i++){

		double angle = i*1.0/(nLines) *(angleMax - angleMin) + angleMin;

		

		lineOffsets[i] = offsetCounter;

		short * linePointer = lines + offsetCounter;


		if(i % 2 == 0){
			
			ax = (short) (cos(angle)*iRadius + mx);
			ay = (short) (sin(angle)*iRadius + my);

			ex = (short) (cos(angle)*(double) oRadius + (double) mx);
			ey = (short) (sin(angle)*(double) oRadius + (double) my);


			int offset = DrawLine(linePointer, ax, ay, ex, ey);
			offsetCounter += 2*offset;

		}
		else{

			ax = (short) (cos(angle)*(iRadius + (oRadius - iRadius)/4) + mx);
			ay = (short) (sin(angle)*(iRadius + (oRadius - iRadius)/4) + my);

			ex = (short) (cos(angle)*oRadius + mx);
			ey = (short) (sin(angle)*oRadius + my);
			
			int offset = DrawLine(linePointer, ax, ay, ex, ey);
			offsetCounter += 2*offset;
		}



	}

	lineOffsets[nLines] = offsetCounter;

	printf("ScanLineHelper Ball Init End\n");

}

int ScanLineHelperDirected::DrawLine(short * line, short ax, short ay, short ex, short ey){

	short x = ax;
	short y = ay;
	short D = 0;
	short HX = ex - ax;
	short HY = ey - ay;
	short xInc = 1;
	short yInc = 1;

	int pointCounter = 0;

	printf(" Scan Line Helper Ball ax: %d ay: %d ex: %d ey: %d\n", ax, ay, ex, ey);

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
		int counter = 0;
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
		pointCounter = counter;
	}	
	else {

		short c = 2 * HY; 
		short M = 2 * HX;
		int counter = 0;
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
		pointCounter = counter;
	}

	printf("Size of line: %d\n", pointCounter);

	return pointCounter;

}



short * ScanLineHelperDirected::getLines(){

	return lines;

}


int * ScanLineHelperDirected::getLineOffsets(){

	return lineOffsets;

}



short ScanLineHelperDirected::getNumberLines(){

	return nLines;

}


short ScanLineHelperDirected::getMaxPoints(){

	return maxPoints;

}

