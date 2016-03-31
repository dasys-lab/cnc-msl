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
#include "FilterTemplateMatching.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>

using namespace std;

const float FilterTemplateMatching::pi = 3.14159265;

FilterTemplateMatching::FilterTemplateMatching(int width, int height):Filter(OF_GRAY, width, height){

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

	this->width = width;
	this->height = height;

	this->sc = SystemConfig::getInstance();
	Configuration *loc = (*this->sc)["ROI"];

        Bx1 = (int)loc->get<int>("ROI", "Kicker1X", NULL);
        By1 = (int)loc->get<int>("ROI", "Kicker1Y", NULL);

        Bx2 = (int)loc->get<int>("ROI", "Kicker2X", NULL);
        By2 = (int)loc->get<int>("ROI", "Kicker2Y", NULL);

        Bx3 = (int)loc->get<int>("ROI", "Kicker3X", NULL);
        By3 = (int)loc->get<int>("ROI", "Kicker3Y", NULL);

	Configuration *kh = (*this->sc)["KickHelper"];
	this->kickerCount = (int)kh->tryGet<int>(3, "KickConfiguration", "KickerCount", NULL);

    Configuration *vision = (*sc)["Vision"];
    duelBlackCountThreshold = vision->tryGet<int>(800, "Vision", "DuelBlackCountThreshold", NULL);

	init(width, height);

}



FilterTemplateMatching::~FilterTemplateMatching(){
	cleanup();
}
		
int inline FilterTemplateMatching::sign2(int s) {
	return (s<0) ? -2 : 2;
}

int inline FilterTemplateMatching::sign(int s) {
        return (s<0) ? -1 : 1;
}

int inline FilterTemplateMatching::abs(int val) {
	return (val<0) ? -val : val;
}

float inline FilterTemplateMatching::dir(int gx, int gy, int threshold) {
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

int FilterTemplateMatching::ballInKickerTest(unsigned char * src, int kickerNum, int* &ballb, int& ballCount, int bc) {
	int sum, ist, index, minT, maxT, lookUpPos=0;
	int ret=0;
	int err;
	switch(kickerNum) {
		case 0:
			minT = 3;
			maxT = 8;
			err = 4; //3 ?
			break;
		case 1:
			minT = 6;
			maxT = 6+6;
			err = 4; // 3?
			break;
		case 2:
			minT = 10;
			maxT = 10+5;
			err = 4; //3 ?
			
			break;
		default:
			return false;
	}

	int inc=3;
	int maxPoints = extendedPoints;
	if(kickerNum==1) maxPoints = kickersPoints;
	for(int k=kickerNum; k<maxPoints; k+=inc) {
		if(k>kickersPoints) { 
			inc = 2;
			err = 2;
		}
		//cout << k << " " << maxPoints << " " << kickersPoints << " " << ballKickerPos[k] <<  endl;
		index = ballKickerPos[k];
		//src[index]=255;
		for(int i=15; i<18; i++) {
			sum=err;
			for(int n=maxT; n>=minT; --n) {
				//ist = src[index + templateLookUp[i][n%12][0]];
				lookUpPos = i*COFFSET + (n%12)*3;
				ist = src[index + templateLookUp[lookUpPos]];

				//src[index + templateLookUp[lookUpPos]] = 255;
				//src[index + templateLookUp[lookUpPos]+1] = 255;
				//src[index + templateLookUp[lookUpPos]-1] = 255;

				if(ist==0) {
					if(--sum == 0) break;
					continue;
				}
				//70
				if(abs(templateLookUp[lookUpPos + 2] - ist) > 70) sum--;
				
				if(sum==0) break;
			}
			if(sum>0) {
				if(ballCount < MAXBALLNUM) {
					ballb[bc++] = index%width;
					ballb[bc++] = index/width;
					ballb[bc++] = i;
					ballb[bc++] = sum;
					ballCount++;
				}
				ret = index;
				//return index;
			}

		}
	}
	return ret;
}

unsigned char * FilterTemplateMatching::process(unsigned char * src, int* &ballb, int& ballCount,unsigned char * mask, int maskThresh ,int width, int height, int minRad, int maxRad, int threshold) {
	//unsigned char * tgt = outputBuffer;

	int sum=0;
	int err=12+1-threshold;
	int minBallSize=minRad;
	int ist;
	int index;
	int lookUpPos;
	//int Bx1 = 202, Bx2 = 233, Bx3 = 263, By1 = 265, By2 = 209, By3 = 265;

	ballb = balls;

	ballCount = 0;
	int bc = 0;

	for(int y = maxRad+1; y < height - (maxRad+1); ++y){
		for(int x = maxRad+1; x < width - (maxRad+1); ++x){			
			index = y*width+x;

			 //Template Demo
			 /*
			 if(index==100+100*width) {
			 for(int n=9; n<=9+7; n++) {
			  src[index + templateLookUp[10*COFFSET +(n%12)*3] + index] = 255;
			  src[index + templateLookUp[10*COFFSET +(n%12)*3+1] + index] = 255;
			  src[index + templateLookUp[10*COFFSET +(n%12)*3-1] + index] = 255;
			 }
			}*/
			
			if(mask[index]<maskThresh) continue;
			if(abs(x-233) < 130 && abs(y-233) < 130) {
				minBallSize = ((((abs(x-233)+abs(y-233))/2)*-10)+1600)/100;
			} 
			else {
				minBallSize = minRad;
			}

			//for(int i=minRad; i<=maxRad; i+= (i<19)?1:2) {
			for(int i=minBallSize; i<=maxRad; i++) {
				sum = err;
				for(int n=11; n>=0; --n) {
					lookUpPos = i*COFFSET +n*3;
					ist = src[index + templateLookUp[lookUpPos]];
					if(ist==0) {
						if(--sum == 0) break;
						continue;
					}
					//if(abs(templateLookUp[i][n][2] - ist) > 70) sum--;
					//70
					if(abs(templateLookUp[ lookUpPos + 2 ] - ist) > 70) if(--sum == 0) break;
				}
				if(sum>0) {
					if(ballCount < MAXBALLNUM) {
						balls[bc++] = x;
						balls[bc++] = y;
						balls[bc++] = i;
						balls[bc++] = sum;
					}
					ballCount++;
					//tgt[index] = 255; 
					//tgt[index+1] = 255;
					//tgt[index-1] = 255;
				}
				//else tgt[index+1] = 0;
			}
			
		}
	}

	if(kickerCount>1) {
		if(ballInKickerTest(src, 0, ballb, ballCount, bc)) {
		return outputBuffer;
		}
	}
	if(kickerCount>0) {
		if(ballInKickerTest(src, 1, ballb, ballCount, bc)) {
			return outputBuffer;
		}
	}
	if(kickerCount>2) {
		ballInKickerTest(src, 2, ballb, ballCount, bc);
	}
	return outputBuffer;
}


int avgROIInt(ROIData dat, int width, unsigned char * gray, unsigned char thresh) {
	long long ret=0, count=0;
	for(int x=dat.left; x<dat.right; x++) {
		for(int y=dat.top; y<dat.bottom; y++) {
            if(gray[x+y*width]>thresh)
			count++;
		}
	}
	//if(count == 0) return 0;
	return count;
}


unsigned char * FilterTemplateMatching::process(unsigned char * src, int* &ballb, int& ballCount, unsigned char * mask, std::vector<ROIData> &roiData, int maskThresh, int width, int height, int minRad, int maxRad, int threshold, unsigned char * gray) {
	//unsigned char * tgt = outputBuffer;

	int sum=0;
	int err=12+1-threshold;
	int minBallSize=minRad;
	int ist;
	int index;
	int lookUpPos;
	//int Bx1 = 202, Bx2 = 233, Bx3 = 263, By1 = 265, By2 = 209, By3 = 265;

	//int minBallSize=minRad;
	//int minballSizeROI=minRad;
	int maxballSizeROI=maxRad;
	int maxballSizeROIx=maxRad;


	ballb = balls;
	ROIData dat;

	ballCount = 0;
	int bc = 0;

	for(int n=0; n<roiData.size()-kickerCount; n++){
		dat = roiData[n];


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
			if(abs(x-233) < 130 && abs(y-233) < 130) {
				minBallSize = ((((abs(x-233)+abs(y-233))/2)*-10)+1600)/100;
			} 
			else {
				minBallSize = minRad;
			}
			if(minBallSize>4) minBallSize--;

			//for(int i=minRad; i<=maxRad; i+= (i<19)?1:2) {
			for(int i=minBallSize; i<=maxRad; i++) {
				sum = err;
				for(int n=11; n>=0; --n) {
					lookUpPos = i*COFFSET +n*3;
					ist = src[index + templateLookUp[lookUpPos]];
					if(ist==0) {
						if(--sum == 0) break;
						continue;
					}
					//if(abs(templateLookUp[i][n][2] - ist) > 70) sum--;
					//70
					if(abs(templateLookUp[ lookUpPos + 2 ] - ist) > 70) if(--sum == 0) break;
				}
				if(sum>0) {
					if(ballCount < MAXBALLNUM) {
						balls[bc++] = x;
						balls[bc++] = y;
						balls[bc++] = i;
						balls[bc++] = sum;
					}
					ballCount++;
					//tgt[index] = 255; 
					//tgt[index+1] = 255;
					//tgt[index-1] = 255;
				}
				//else tgt[index+1] = 0;
			}
		    }
		}
	}

    //cout << "AvgROIInt: "<< avgROIInt(roiData[roiData.size()-3], width, gray) << " X " << roiData[roiData.size()-3].midX << " Y " << roiData[roiData.size()-3].midY << " " <<  avgROIInt(roiData[roiData.size()-2], width, gray) << " " <<  avgROIInt(roiData[roiData.size()-1], width, gray) << endl;
    //cout << "Blackdots in ROI: "<<  (26*57)-avgROIInt(roiData[roiData.size()-1], width, gray, 50)<< endl;
    SpicaHelper::duel = -1;
    SpicaHelper::haveBall = false;
    //26*57 = ROI Area see FilterLinePointsROI Constructor
    if((26*57)-avgROIInt(roiData[roiData.size()-1], width, gray, 50)>duelBlackCountThreshold) {
        SpicaHelper::duel = true;
    } else {
        SpicaHelper::duel = false;
    }

    if(avgROIInt(roiData[roiData.size()-3], width, gray, 80)>80 && kickerCount>1) {
		if(ballInKickerTest(src, 0, ballb, ballCount, bc)) {
            SpicaHelper::haveBall = true;
			return outputBuffer;
		}
	}
	if(kickerCount>1) {
        if(avgROIInt(roiData[roiData.size()-(2)], width, gray, 80)>80) {
			if(ballInKickerTest(src, 1, ballb, ballCount, bc)) {
                SpicaHelper::haveBall = true;
				return outputBuffer;
			}
		}
	} else if (kickerCount ==1) {
        if(avgROIInt(roiData[roiData.size()-1], width, gray, 80)>80) {
                if(ballInKickerTest(src, 1, ballb, ballCount, bc)) {
                        SpicaHelper::haveBall = true;
                        return outputBuffer;
                }
        }
	}
    if(avgROIInt(roiData[roiData.size()-1], width, gray, 80)>80 && kickerCount>2) {
        if(ballInKickerTest(src, 2, ballb, ballCount, bc)) {
            SpicaHelper::haveBall = true;
        }
	}
	return outputBuffer;
}






void FilterTemplateMatching::init(int width, int height){
	float deg;
	int rx, ry;
	balls = (int*) malloc(MAXBALLNUM * B_SIZE * sizeof(int));
	ballKickerPos = (int*) malloc(3000*sizeof(int));
	
	int B1 = Bx1 + By1*width;
	int B2 = Bx2 + By2*width;
	int B3 = Bx3 + By3*width;

	kickersPoints = 0;


	for(int x=-14; x<15; x++) {
		for(int y=-13; y<14; y++) {
			ballKickerPos[kickersPoints++] = B1 + (x-5) + (y)*width; 
			ballKickerPos[kickersPoints++] = B3 + (x+5) + (y)*width;
		}
	}

	for(int x=-14; x<15; x++) {
		for(int y=-13; y<14; y++) {
			ballKickerPos[kickersPoints++] = B2 + (y) + (x-8)*width;
		}
	}


	extendedPoints = kickersPoints;
	//Extended Kickerspositions
	
        for(int x=-6; x<8; x++) {
		for(int y=6; y<26; y++) {
			ballKickerPos[extendedPoints++] = B1 + (x-8) + (y)*width;
			ballKickerPos[extendedPoints++] = B3 + (x+7) + (y)*width;
			//cout << "blub " << extendedPoints << " " << ballKickerPos[extendedPoints-1] << " " << ballKickerPos[extendedPoints-2] << endl;
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
}


void FilterTemplateMatching::cleanup(){
	if(AreaLookup != NULL)
		free(AreaLookup);

	free(balls);
}

