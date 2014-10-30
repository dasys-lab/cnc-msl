/*
 * $Id: FilterSobelGradient.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "FilterTemplateMatchingGoalie.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>



using namespace std;

FilterTemplateMatchingGoalie::FilterTemplateMatchingGoalie(int width, int height):Filter(OF_GRAY, width, height){

	AreaLookup = (unsigned char *) malloc(width*height*sizeof(unsigned char));

	FILE * areafile = fopen("ReduceAreaLookup.dat", "r");
	if(areafile != NULL){

		int n = fread(AreaLookup, sizeof(char), width*height, areafile);
		fclose(areafile);
		printf("AreaLookup: %d bytes read\n", n);
	}
	else{
		printf("Area Lookup File not found\n");

	}
	init(width, height);

}



FilterTemplateMatchingGoalie::~FilterTemplateMatchingGoalie(){
	cleanup();
}
		
int inline FilterTemplateMatchingGoalie::sign2(int s) {
	return (s<0) ? -2 : 2;
}

int inline FilterTemplateMatchingGoalie::sign(int s) {
        return (s<0) ? -1 : 1;
}

int inline FilterTemplateMatchingGoalie::abs(int val) {
	return (val<0) ? -val : val;
}

float inline FilterTemplateMatchingGoalie::dir(int gx, int gy, int threshold) {
	float grad;
	if(gx*gx + gy*gy > threshold) {
		//if(gx == 0)
		//	grad = gy;
		if(gy == 0)
			grad = sign2(gy);
		else {
			grad = ((float)gx)/((float)gy);
		}
	
		if (grad > 2) grad = 2;
		else if(grad < -2) grad = -2;
		grad = ((grad+2)*60)+1;
	}
	else {
		grad = 0;
	}
	return grad;
}

bool FilterTemplateMatchingGoalie::ballInKickerTest(unsigned char * src, int kickerNum) {
	int sum, ist, index, minT, maxT, lookUpPos=0;
	int err;
	switch(kickerNum) {
		case 0:
			minT = 3;
			maxT = 8;
			err = 2;
			break;
		case 1:
			minT = 6;
			maxT = 6+6;
			err = 3;
			break;
		case 2:
			minT = 10;
			maxT = 10+5;
			err = 2;
			break;
		default:
			return false;
	}

	for(int k=kickerNum; k<kickersPoints; k+=3) {
		index = ballKickerPos[k];
		for(int i=15; i<16; i++) {
			sum=err;
			for(int n=maxT; n>=minT; --n) {
				//ist = src[index + templateLookUp[i][n%12][0]];
				lookUpPos = i*COFFSET + (n%12)*3;
				ist = src[index + templateLookUp[lookUpPos]];

				//src[index + templateLookUp[i][n%12][0]] = 255;
				//src[index + templateLookUp[i][n%12][0]+1] = 255;
				//src[index + templateLookUp[i][n%12][0]-1] = 255;
				if(ist==0) {
					if(--sum == 0) break;
					continue;
				}
				if(abs(templateLookUp[lookUpPos + 2] - ist) > 70) sum--;
				
				if(sum==0) break;
			}
			if(sum>0) return true;

		}
	}
	return false;
}

inline int max(int a, int b) {
	return (a>b) ? a : b;
}

inline int min(int a, int b) {
	return (a<b) ? a : b;
}

unsigned char * FilterTemplateMatchingGoalie::process(unsigned char * src, int* &ballb, int& ballCount,unsigned char * mask, std::vector<ROIData> &roiData, int maskThresh, int width, int height, int minRad, int maxRad, int threshold, unsigned char * gray) {
	//unsigned char * tgt = outputBuffer;

	register int sum=0;
	int err=12+1-threshold;
	int minT=8;//8;
	int maxT=8+8;//8+8;
	int minBallSize=minRad;
	int minballSizeROI=minRad;
	int maxballSizeROI=maxRad;
	int maxballSizeROIx=maxRad;

	int ist;
	register int index;
	int lookUpPos;
	//int Bx1 = 202, Bx2 = 233, Bx3 = 263, By1 = 265, By2 = 209, By3 = 265;

	ballb = balls;
	ROIData dat;

	ballCount = 0;
	int bc = 0;

			 //Template Demo
		/*	 
			 index=100+100*width;
			 for(int n=minT; n<=maxT; n++) {
			  src[index + templateLookUp[10*COFFSET +(n%12)*3] + index] = 255;
			  src[index + templateLookUp[10*COFFSET +(n%12)*3] + index+1] = 255;
			  src[index + templateLookUp[10*COFFSET +(n%12)*3] + index-1] = 255;
			 }
			 index=0;
		*/

//	for(int y = maxRad+1; y < height - (maxRad+1); ++y){
//		for(int x = maxRad+1; x < width - (maxRad+1); ++x){			
	for(int i=0; i<roiData.size(); i++){
		dat = roiData[i];

		if(dat.right-dat.left > maxRad*2 && dat.bottom-dat.top > maxRad*2) {
			long long sum=0;
			int count=0;
			for(int x=dat.left+minRad; x<dat.right-minRad; x++) {
				for(int y=dat.top+minRad; y<dat.bottom-minRad; y++) {
					count++;
					sum += src[y*width+x];
				}
			}
			if(sum/count > 100) {
				if(ballCount < MAXBALLNUM) {
					balls[bc++] = dat.midX;
					balls[bc++] = dat.midY;
					balls[bc++] = min(dat.right-dat.left, dat.bottom-dat.top);
					balls[bc++] = 1;
					cout << "MonsterROI: " << endl;
				}
				ballCount++;
				break;
			}
		}

		for(int x=dat.left+minRad; x<dat.right-minRad; x++) {
		    maxballSizeROIx = x-dat.left;
		    maxballSizeROIx = min(maxballSizeROIx, dat.right-x);

		    for(int y=dat.top+minRad; y<dat.bottom-minRad; y++) {
			maxballSizeROI = min(maxballSizeROIx, y-dat.top);
			maxballSizeROI = min(maxballSizeROI, dat.bottom-y);
			//maxballSizeROI = maxballSizeROIx;
			maxballSizeROI = min(maxRad, maxballSizeROIx);

			index = y*width+x;

			/* Template Demo
			 if(index==100+100*width) {
			 for(int n=0; n<12; n++) {
			  tgt[index + templateLookUp[10][n][0] + (templateLookUp[10][n][1]*width)] = templateLookUp[10][n][2];
			 }
			}
			*/
			if(mask[index]<maskThresh) continue;

			minBallSize = YtoMinBallLookUp[x];
			if(minBallSize < minRad) minBallSize = minRad;

			//for(int i=minRad; i<=maxRad; i+= (i<19)?1:2) {
			for(register int i=minBallSize; i<=maxballSizeROI; i+=2) {
				sum = err;
				for(register int n=maxT; n>=minT; --n) {
					lookUpPos = i*COFFSET +(n%12)*3;
					ist = src[index + templateLookUp[lookUpPos]];
					if(ist==0) {
						if(--sum == 0) break;
						continue;
					}
					//if(abs(templateLookUp[i][n][2] - ist) > 70) sum--;
					if(abs(templateLookUp[ lookUpPos + 2 ] - ist) > 80) if(--sum == 0) break;
				}
				if(sum>0) {
					int topsum=0, bottomsum=0;
					for(int xo=0; xo<=i/2; xo++) {
						topsum += gray[index-xo];
						topsum += gray[index-xo-width];
						topsum += gray[index-xo+width];
					}
						

					for(int xu=(i/2)+1; xu<=i+1; xu++) {
						bottomsum += gray[index-xu];
						bottomsum += gray[index-xu-width];
						bottomsum += gray[index-xu+width];
					}
					
					if(topsum>bottomsum) 
					{
					if(ballCount < MAXBALLNUM) {
						balls[bc++] = x;
						balls[bc++] = y;
						balls[bc++] = i;
						balls[bc++] = sum;
						cout << "Mask-Val: " << (int)mask[index] << endl;
					}
					ballCount++;
					}
				}
			}
		    }
		}
	}
	return outputBuffer;
}



void FilterTemplateMatchingGoalie::init(int width, int height){
	float deg;
	int rx, ry;
	balls = (int*) malloc(MAXBALLNUM * B_SIZE * sizeof(int));
	
	int B1 = Bx1 + By1*width;
	int B2 = Bx2 + By2*width;
	int B3 = Bx3 + By3*width;

	kickersPoints = 0;

	for(int x=-3; x<4; x++) {
		for(int y=-3; y<4; y++) {
			ballKickerPos[kickersPoints++] = B1 + x + y*width; 
			ballKickerPos[kickersPoints++] = B2 + x + y*width;
			ballKickerPos[kickersPoints++] = B3 + x + y*width;
		}
	}

	for(int radius=2; radius<RADNUM; radius++) {
		for(int d=0; d<CIRCPOINTS; d++) {
			deg=((float)d)*((2*pi)/CIRCPOINTS);
			ry = round(sin(deg)*radius);
			rx = round(cos(deg)*radius);
			
			templateLookUp[radius*COFFSET+d*3] = rx + (ry*width);
			//templateLookUp[radius][d][1] = ry*width;
			templateLookUp[radius*COFFSET+d*3+1] = ry*width;
			templateLookUp[radius*COFFSET+d*3+2] = dir(rx, ry, 0);
			templateLookUp[radius*COFFSET+d*3+2] = abs(templateLookUp[radius*COFFSET+d*3+2]);
		}
	}

	PositionHelperDirected *pHelp = PositionHelperDirected::getInstance();
	YtoMinBallLookUp = new int[width+2];
	for(int i=height+1; i>=0; i--) {
		Point p = pHelp->getPointCam2Field(i, width/2);

		if(p.x > 3*FootballField::FieldLength) {
			YtoMinBallLookUp[i] = 3;
			continue;
		}

		YtoMinBallLookUp[i] = fabs(pHelp->getPoint3D2Radius(p.x, p.y, 0)) - 5;	

		cout << YtoMinBallLookUp[i] << "\t";
	}	
}


void FilterTemplateMatchingGoalie::cleanup(){
	if(AreaLookup != NULL)
		free(AreaLookup);
}

