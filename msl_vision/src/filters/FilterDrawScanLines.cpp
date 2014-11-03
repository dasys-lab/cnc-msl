/*
 * $Id: FilterDrawScanLines.cpp 1987 2007-04-09 16:58:10Z rreichle $
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
#include "FilterDrawScanLines.h"
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <string>



FilterDrawScanLines::FilterDrawScanLines(int width, int height):Filter(OF_ZERO, width, height){

	this->sc = SystemConfig::getInstance();

	Configuration *vision = (*this->sc)["Vision"];

	negRanges[0][0] = vision->get<short>("Vision", "Holder", "NegRange_0_0", NULL);
	negRanges[0][1] = vision->get<short>("Vision", "Holder", "NegRange_0_1", NULL);
	negRanges[1][0] = vision->get<short>("Vision", "Holder", "NegRange_1_0", NULL);
	negRanges[1][1] = vision->get<short>("Vision", "Holder", "NegRange_1_1", NULL);
	negRanges[2][0] = vision->get<short>("Vision", "Holder", "NegRange_2_0", NULL);
	negRanges[2][1] = vision->get<short>("Vision", "Holder", "NegRange_2_1", NULL);

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



FilterDrawScanLines::~FilterDrawScanLines(){

	cleanup();

}
		

unsigned char * FilterDrawScanLines::process(unsigned char * src, unsigned int width, unsigned int height, ScanLineHelper & helper, bool gray){

	unsigned char * tgt = src;

	short * firstInner = helper.getInnerLines();
	short * nInner = helper.getInnerLinesN();

	short * firstOuter = helper.getOuterLines();
	short * nOuter = helper.getOuterLinesN();

	//printf("Number of ScanLines: %d\n", helper.getNumberLines());

	short maxPoints = helper.getMaxPoints();

	short x;
	short y;

	for(short i = 0; i < helper.getNumberLines(); i++){

		bool draw = true;

		if(negRanges[0][0] > helper.getNumberLines()/2 && negRanges[0][1] < helper.getNumberLines()/2){

			if( (i >= negRanges[0][0]) || (i <= negRanges[0][1]) || 
				(i >= negRanges[1][0] && i <= negRanges[1][1]) ||
				(i >= negRanges[2][0] && i <= negRanges[2][1]))
				draw = false;

		}
		else {

			if( (i >= negRanges[0][0] && i <= negRanges[0][1]) || 
				(i >= negRanges[1][0] && i <= negRanges[1][1]) ||
				(i >= negRanges[2][0] && i <= negRanges[2][1]))
				draw = false;

		}

		double lineAngle = i*1.0/helper.getNumberLines()*360.0;

		for(unsigned a = 0; a < addHolders.size(); a++){
			if(addHolders[a].start > 180.0 && addHolders[a].start < 180.0){

				if(lineAngle >= addHolders[a].start || lineAngle <= addHolders[a].end)
					draw = false;

			}
			else {

				if(lineAngle >= addHolders[a].start && lineAngle <= addHolders[a].end)
					draw = false;

			}

		}



		int lineNumber = helper.getNumberLines()/4;
		if(i == lineNumber)
			draw = false;

		if(i % 2 == 0){

			short * line = firstInner;

			if(draw){

				for(short j = 0; j < (*nInner); j++){
					x = *line++;
					y = *line++;
					if(gray){
						src[(x * width + y)] = 255;
					}
					else {
						src[(x * width + y) * 3] = 255;
						src[(x * width + y) * 3 + 1] = 255;
						src[(x * width + y) * 3 + 2] = 255;
					}
				}
			}

			firstInner = firstInner + maxPoints*2;
			nInner++;
	

		}
		else {

			short * line = firstOuter;

			if(draw){
				for(short j = 0; j < (*nOuter); j++){
					x = *line++;
					y = *line++;
	
					if(gray){
						src[(x * width + y)] = 255;
					}
					else {
						src[(x * width + y) * 3] = 255;
						src[(x * width + y) * 3 + 1] = 255;
						src[(x * width + y) * 3 + 2] = 255;
					}
				}

			}

			firstOuter = firstOuter + maxPoints;
			nOuter++;

		}

	}

	short numberOfCircles = helper.getNumberCircles();

	printf("NumberOfCircles = %d\n", numberOfCircles);

	short * circles = helper.getCircles();
	short * circleOffsets = helper.getCircleOffsets();

	printf("Circles : %d\n", circleOffsets[numberOfCircles]);

	for(int i = 0; i < circleOffsets[numberOfCircles]; i++){

		short x = *circles++;
		short y = *circles++;

		//printf("Circles x : %d y : %d\n", x, y);

		if(gray){
			src[(x * width + y)] = 255;
		}
		else {
			src[(x * width + y) * 3] = 255;
			src[(x * width + y) * 3 + 1] = 255;
			src[(x * width + y) * 3 + 2] = 255;
		}


	}


	return tgt;

}


unsigned char * FilterDrawScanLines::process(unsigned char * src, unsigned int width, unsigned int height, ScanLineHelperDirected & helper, bool gray){

	
	unsigned char * tgt = src;

	short * linePtr = helper.getLines();

	for(int i = 0; i < helper.getLineOffsets()[helper.getNumberLines()]; i++){
		short x = *linePtr++;
		short y = *linePtr++;

                if(gray){
                        src[(x * width + y)] = 255;
                }
                else {
                        src[(x * width + y) * 3] = 255;
                        src[(x * width + y) * 3 + 1] = 255;
                        src[(x * width + y) * 3 + 2] = 255;
                }
		
	}

	return tgt;

}




void FilterDrawScanLines::init(){


}


void FilterDrawScanLines::cleanup(){


}

