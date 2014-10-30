/*
 * $Id: BallHelper.cpp 1531 2006-08-01 21:36:57Z phbaer $
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

#include "BallClusterHelp.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <sys/time.h>
#include <iostream>

#define CL_ENTRYS 7
#define B_ENTRYS 4

using namespace std;
        
int BallClusterHelp::clusterBalls(int * balls, int ballCount, ballCluster * cluster, int maxClusterCount) {
	int clusterCount = 0;
	bool clusterFound;
	
	for(int i=0; i < ballCount; i++) {
		int index = i * B_ENTRYS;
		clusterFound = false;

		for(int c=0; c<clusterCount; c++) {
			int cnum = c;
			int xdiff = (cluster[cnum].x - balls[index]);
			int ydiff = (cluster[cnum].y - balls[index+1]);

			if(xdiff*xdiff + ydiff*ydiff <= 16) {
				clusterFound = true;
				if(cluster[cnum].maxRadius < balls[index+2]) 
					cluster[cnum].maxRadius = balls[index+2];

				else if(cluster[cnum].minRadius > balls[index+2]) 
					cluster[cnum].minRadius = balls[index+2];

				cluster[cnum].xballsum += balls[index];
				cluster[cnum].yballsum += balls[index+1];

				cluster[cnum].balls++;
				cluster[cnum].err += balls[index+3];
				cluster[cnum].x = cluster[cnum].xballsum / cluster[cnum].balls;
				cluster[cnum].y = cluster[cnum].yballsum / cluster[cnum].balls;

				cluster[cnum].sizeSum += balls[index + 2];

				break;
			}
		}

		if(!clusterFound) {
			if(clusterCount >= maxClusterCount) break;
			//X
			cluster[clusterCount].x = balls[index];
			//Y
			cluster[clusterCount].y = balls[index + 1];
			//minsize
			cluster[clusterCount].minRadius = balls[index + 2];
			//maxsize
			cluster[clusterCount].maxRadius = balls[index + 2];
			//ballerr Points
			cluster[clusterCount].err = balls[index+3];
			//ballcount
			cluster[clusterCount].balls = 1;
			//xsum
			cluster[clusterCount].xballsum = balls[index];
			//ysum
			cluster[clusterCount].yballsum = balls[index + 1];
			//sum of sizes
			cluster[clusterCount].sizeSum = balls[index + 2];
			clusterCount++;
		}
	}
	//std::cout << ballCount << std::endl;
	return clusterCount;
}


void BallClusterHelp::clusterStdOut(ballCluster * cluster, int clusterCount, int xmid, int ymid, bool dist) {
	for(int i=0; i<clusterCount; i++) {
		int pos = i;
		if(!dist) {
			cout << "endyball " << "x: " << cluster[pos].x << "\ty: " << cluster[pos].y << "\tminBallSize: " << cluster[pos].minRadius << "\tmaxBallSize: " << cluster[pos].maxRadius << endl;
		}
		else {
			int xdiff = fabs(cluster[pos].x-xmid);
			int ydiff = fabs(cluster[pos].y-ymid);

			cout << "endyball " << sqrt(xdiff*xdiff + ydiff*ydiff) << " " << cluster[pos].maxRadius << " " << cluster[pos].minRadius << endl;
		}
	}
}


void BallClusterHelp::visualizeCluster(unsigned char *src, int width, int height, ballCluster * cluster, int clusterCount) {
	ballCluster *bestBall=NULL;
	int minError=12;
	for(int i=0; i<clusterCount; i++) {
		int pos = i;
		int irad = cluster[i].sizeSum / cluster[i].balls;
		bool valid=true;

		for(int n=0; n<clusterCount; n++) {
			if(n==i) continue;
			int nrad = cluster[n].sizeSum / cluster[n].balls;
			int xdist = cluster[i].x - cluster[n].x;
			int ydist = cluster[i].y - cluster[n].y;

			if(xdist < 0) xdist = -xdist;
			if(ydist < 0) ydist = -ydist;

			if(nrad>irad && (xdist*ydist < nrad*nrad)) {
				valid=false;
			}			
		}
		if(!valid)
			continue;

		int index = cluster[pos].x + width*cluster[pos].y;

		src[index] = 255;
		src[index+1] = 255;
		src[index-1] = 255;
		src[index+2] = 255;
		src[index-2] = 255;
		src[index+3] = 255;
		src[index-3] = 255;

		src[index+1*width] = 255;
		src[index-1*width] = 255;

		src[index+2*width] = 255;
		src[index-2*width] = 255;

		src[index+3*width] = 255;
		src[index-3*width] = 255;
		drawCircle(src, cluster[pos], width, height, (255*(cluster[pos].err/cluster[pos].balls)/2)%256 );
		if(minError>cluster[pos].err / cluster[pos].balls) {
			minError = cluster[pos].err / cluster[pos].balls;
			bestBall = &cluster[pos];
		}
	}
	if(bestBall != NULL) {
		int index = bestBall->x + width*bestBall->y;
		src[index-1-width] = 255;
		src[index+1+width] = 255;
		src[index+1-width] = 255;
		src[index-1+width] = 255;
		src[index] = 0;
	}
}




void BallClusterHelp::drawCircle(unsigned char *src, ballCluster &b, int width, int height, int Intens) {


//	short radius = b.sizeSum / b.balls;
	short radius = b.maxRadius;

	short currX = 0;
	short currY = 0;

	currX = b.x + radius;
	currY = b.y;

	//segment 1
	while(currX >= b.x && currY >= b.y - radius){

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

					double f = fabs((cX - b.x)*(cX - b.x) + (cY - b.y)*(cY - b.y) - radius*radius*1.0);
	
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

		if(currX < 0 || currX >= width || currY < 0 || currY >= height){
			printf("Circles out of bounds!\n");
		}

		src[currX+currY*width] = Intens;

	}

	//segment 2
	while(currX >= b.x - radius && currY <= b.y){

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

					double f = fabs((cX - b.x)*(cX - b.x) + (cY - b.y)*(cY - b.y) - radius*radius*1.0);
	
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

		src[currX+currY*width]=Intens;
	}



	while(currX <= b.x && currY <= b.y + radius){

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

					double f = fabs((cX - b.x)*(cX - b.x) + (cY - b.y)*(cY - b.y) - radius*radius*1.0);
	
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

		src[currX+currY*width]=Intens;
	}



	while(currX <= b.x + radius && currY >= b.y){

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

					double f = fabs((cX - b.x)*(cX - b.x) + (cY - b.y)*(cY - b.y) - radius*radius*1.0);
	
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

		src[currX+currY*width]=Intens;
	}

}
