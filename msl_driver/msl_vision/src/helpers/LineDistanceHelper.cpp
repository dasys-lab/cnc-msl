/*
 * $Id: LineDistanceHelper.cpp 1874 2007-03-02 20:35:47Z rreichle $
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
#include "LineDistanceHelper.h"

#include "FootballField.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>


LineDistanceHelper::LineDistanceHelper(){

	LineLookup = NULL;
	init();


}


LineDistanceHelper::~LineDistanceHelper(){


	cleanup();


}


void LineDistanceHelper::init(){

	

	LineLookup = (unsigned char *) malloc(IWIDTH*IHEIGHT*sizeof(double));

	FootballField::getInstance();

	int NUMBERLINES = 17;
	int NUMBER_USED_LINES = (FootballField::GoalInnerAreaExists ? 17 : 11);
	
	double Lines[NUMBERLINES][2][2];

	Lines[0][0][0] = FootballField::FieldLength/2.0;
	Lines[0][0][1] = FootballField::FieldWidth/2.0;
	Lines[0][1][0] = FootballField::FieldLength/2.0;
	Lines[0][1][1] = -FootballField::FieldWidth/2.0;

	Lines[1][0][0] = -FootballField::FieldLength/2.0;
	Lines[1][0][1] = FootballField::FieldWidth/2.0;
	Lines[1][1][0] = -FootballField::FieldLength/2.0;
	Lines[1][1][1] = -FootballField::FieldWidth/2.0;

	Lines[2][0][0] = FootballField::FieldLength/2.0;
	Lines[2][0][1] = FootballField::FieldWidth/2.0;
	Lines[2][1][0] = -FootballField::FieldLength/2.0;
	Lines[2][1][1] = FootballField::FieldWidth/2.0;

	Lines[3][0][0] = FootballField::FieldLength/2.0;
	Lines[3][0][1] = -FootballField::FieldWidth/2.0;
	Lines[3][1][0] = -FootballField::FieldLength/2.0;
	Lines[3][1][1] = -FootballField::FieldWidth/2.0;

	Lines[4][0][0] = FootballField::FieldLength/2.0 - FootballField::GoalAreaWidth;
	Lines[4][0][1] = FootballField::GoalAreaLength/2.0;
	Lines[4][1][0] = FootballField::FieldLength/2.0 - FootballField::GoalAreaWidth;
	Lines[4][1][1] = -FootballField::GoalAreaLength/2.0;

	Lines[5][0][0] = FootballField::FieldLength/2.0;
	Lines[5][0][1] = FootballField::GoalAreaLength/2.0;
	Lines[5][1][0] = FootballField::FieldLength/2.0 - FootballField::GoalAreaWidth;
	Lines[5][1][1] = FootballField::GoalAreaLength/2.0;

	Lines[6][0][0] = FootballField::FieldLength/2.0;
	Lines[6][0][1] = -FootballField::GoalAreaLength/2.0;
	Lines[6][1][0] = FootballField::FieldLength/2.0 - FootballField::GoalAreaWidth;
	Lines[6][1][1] = -FootballField::GoalAreaLength/2.0;

	Lines[7][0][0] = -FootballField::FieldLength/2.0 + FootballField::GoalAreaWidth;
	Lines[7][0][1] = FootballField::GoalAreaLength/2.0;
	Lines[7][1][0] = -FootballField::FieldLength/2.0 + FootballField::GoalAreaWidth;
	Lines[7][1][1] = -FootballField::GoalAreaLength/2.0;

	Lines[8][0][0] = -FootballField::FieldLength/2.0;
	Lines[8][0][1] = FootballField::GoalAreaLength/2.0;
	Lines[8][1][0] = -FootballField::FieldLength/2.0 + FootballField::GoalAreaWidth;
	Lines[8][1][1] = FootballField::GoalAreaLength/2.0;

	Lines[9][0][0] = -FootballField::FieldLength/2.0;
	Lines[9][0][1] = -FootballField::GoalAreaLength/2.0;
	Lines[9][1][0] = -FootballField::FieldLength/2.0 + FootballField::GoalAreaWidth;
	Lines[9][1][1] = -FootballField::GoalAreaLength/2.0;

	Lines[10][0][0] = 0.0;
	Lines[10][0][1] = -FootballField::FieldWidth/2.0;
	Lines[10][1][0] = 0.0;
	Lines[10][1][1] = FootballField::FieldWidth/2.0;

	Lines[11][0][0] = FootballField::FieldLength/2.0 - FootballField::GoalInnerAreaWidth;
	Lines[11][0][1] = FootballField::GoalInnerAreaLength/2.0;
	Lines[11][1][0] = FootballField::FieldLength/2.0 - FootballField::GoalInnerAreaWidth;
	Lines[11][1][1] = -FootballField::GoalInnerAreaLength/2.0;

	Lines[12][0][0] = FootballField::FieldLength/2.0;
	Lines[12][0][1] = FootballField::GoalInnerAreaLength/2.0;
	Lines[12][1][0] = FootballField::FieldLength/2.0 - FootballField::GoalInnerAreaWidth;
	Lines[12][1][1] = FootballField::GoalInnerAreaLength/2.0;

	Lines[13][0][0] = FootballField::FieldLength/2.0;
	Lines[13][0][1] = -FootballField::GoalInnerAreaLength/2.0;
	Lines[13][1][0] = FootballField::FieldLength/2.0 - FootballField::GoalInnerAreaWidth;
	Lines[13][1][1] = -FootballField::GoalInnerAreaLength/2.0;

	Lines[14][0][0] = -FootballField::FieldLength/2.0 + FootballField::GoalInnerAreaWidth;
	Lines[14][0][1] = FootballField::GoalInnerAreaLength/2.0;
	Lines[14][1][0] = -FootballField::FieldLength/2.0 + FootballField::GoalInnerAreaWidth;
	Lines[14][1][1] = -FootballField::GoalInnerAreaLength/2.0;

	Lines[15][0][0] = -FootballField::FieldLength/2.0;
	Lines[15][0][1] = FootballField::GoalInnerAreaLength/2.0;
	Lines[15][1][0] = -FootballField::FieldLength/2.0 + FootballField::GoalInnerAreaWidth;
	Lines[15][1][1] = FootballField::GoalInnerAreaLength/2.0;

	Lines[16][0][0] = -FootballField::FieldLength/2.0;
	Lines[16][0][1] = -FootballField::GoalInnerAreaLength/2.0;
	Lines[16][1][0] = -FootballField::FieldLength/2.0 + FootballField::GoalInnerAreaWidth;
	Lines[16][1][1] = -FootballField::GoalInnerAreaLength/2.0;


	for(int x = 0; x < IHEIGHT; x++){
		for(int y = 0; y < IWIDTH; y++){

			double xReal = (y - IWIDTH/2.0)*RESOLUTION;
			double yReal = -(x - IHEIGHT/2.0)*RESOLUTION;

			double p[2];
			p[0] = xReal;
			p[1] = yReal;

//			printf("Point: [%f %f]\n", p[0], p[1]);

			double minDistance = undefined;

			double d = undefined;

			for(int i = 0; i < NUMBER_USED_LINES; i++){
				
				double rv[2];
				rv[0] = Lines[i][1][0] - Lines[i][0][0];
				rv[1] = Lines[i][1][1] - Lines[i][0][1];

				double nv[2];
				nv[0] = -rv[1];
				nv[1] = rv[0];

				double m = 0.5;
				int sp = 0;

//				printf("nv: [%f %f]\n", nv[0], nv[1]);

				if(fabs(nv[0]) > 1E-10){
					double nen = (nv[1]/nv[0] * rv[0] - rv[1]);
//					printf("nen: %f\n", nen);
					if(fabs(nen) > 1E-10){
						m = (Lines[i][0][1] - p[1] - (nv[1]/nv[0]*(Lines[i][0][0] - p[0])))/nen;
						sp = 1;

					}

				}
				else if(fabs(nv[1]) > 1E-10) {
					double nen = (nv[0]/nv[1]*rv[1] - rv[0]);
					if(fabs(nen) > 1E-10){
						m = (Lines[i][0][0] - p[0] - (nv[0]/nv[1]*(Lines[i][0][0] - p[0])))/nen;
						sp = 1;
					}
				}

				d = undefined;

//				printf("m: %f\n", m);

				if(m < 0.0){
					d = sqrt((p[0] - Lines[i][0][0])*(p[0] - Lines[i][0][0]) + (p[1] - Lines[i][0][1])*(p[1] - Lines[i][0][1]));
				}

				if(m > 1.0){
					d = sqrt((p[0] - Lines[i][1][0])*(p[0] - Lines[i][1][0]) + (p[1] - Lines[i][1][1])*(p[1] - Lines[i][1][1]));
				}

				if( m >= 0.0 && m <= 1.0 && sp == 1){

					double SPoint[2];
					SPoint[0] = Lines[i][0][0] + m * rv[0];
					SPoint[1] = Lines[i][0][1] + m * rv[1];

					d = sqrt((p[0] - SPoint[0])*(p[0] - SPoint[0]) + (p[1] - SPoint[1])*(p[1] - SPoint[1]));


				}

				if(d < minDistance)
					minDistance = d;

			}

			d = fabs(sqrt(p[0]*p[0] + p[1]*p[1]) - FootballField::MiddleCircleRadius);
			if(d < minDistance)
				minDistance = d;

			if(FootballField::CornerCircleExists && fabs(p[0]) < FootballField::FieldLength/2.0 && fabs(p[1]) < FootballField::FieldWidth/2.0){

				double xCoord = p[0] - FootballField::FieldLength/2.0;
				double yCoord = p[0] - FootballField::FieldWidth/2.0;

				d = fabs(sqrt(xCoord*xCoord + yCoord*yCoord) - FootballField::CornerCircleRadius);
				if(d < minDistance)
					minDistance = d;

				xCoord = p[0] + FootballField::FieldLength/2.0;
				yCoord = p[0] - FootballField::FieldWidth/2.0;

				d = fabs(sqrt(xCoord*xCoord + yCoord*yCoord) - FootballField::CornerCircleRadius);
				if(d < minDistance)
					minDistance = d;

				xCoord = p[0] - FootballField::FieldLength/2.0;
				yCoord = p[0] + FootballField::FieldWidth/2.0;

				d = fabs(sqrt(xCoord*xCoord + yCoord*yCoord) - FootballField::CornerCircleRadius);
				if(d < minDistance)
					minDistance = d;

				xCoord = p[0] + FootballField::FieldLength/2.0;
				yCoord = p[0] + FootballField::FieldWidth/2.0;

				d = fabs(sqrt(xCoord*xCoord + yCoord*yCoord) - FootballField::CornerCircleRadius);
				if(d < minDistance)
					minDistance = d;


			}

			double tribot = 1 - (250*250/(minDistance*minDistance + 250*250));

			unsigned int tribot_real = (unsigned int) lrint(tribot*255.0);
			
			unsigned int mDist_real = lrint(minDistance/10.0);
			unsigned char mDist = 0;

			if(mDist_real > 255)
				mDist = 255;
			else
				mDist = (unsigned char) mDist_real;

//			LineLookup[x*IWIDTH + y] = mDist;
			LineLookup[x*IWIDTH + y] = (unsigned char) tribot_real;
//			printf("Distance of Point1: %f\n", minDistance);
		}
	}

	/*FILE * ldfile = fopen("LineDistances.txt", "w");
	for(int i = 0; i < IHEIGHT; i++){
		for(int j = 0; j < IWIDTH; j++){
			fprintf(ldfile, "%d ", LineLookup[i*IWIDTH + j]); 
		}
		fprintf(ldfile, "\n");
	}
	fclose(ldfile);
	*/
}


void LineDistanceHelper::cleanup(){

	if(LineLookup != NULL)
		free(LineLookup);


}


unsigned char LineDistanceHelper::getLineDistance(double px, double py){

	int indX = (int) rint(-py/RESOLUTION + IHEIGHT/2.0);
	int indY = (int) rint(px/RESOLUTION + IWIDTH/2.0);

	printf("IndX %d IndY %d\n", indX, indY);

	if(indX >= 0 && indX < IHEIGHT && indY >= 0 && indY < IWIDTH)
		return LineLookup[indX*IWIDTH + indY];
	else
		return MAX_LDIST;




}


unsigned char * LineDistanceHelper::getLineLookup(){

	return LineLookup;

}
