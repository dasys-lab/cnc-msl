/*
 * $Id: FilterLinePointsROI.cpp 1935 2007-03-19 19:50:12Z phbaer $
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
 *
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */
#include "FilterLinePointsROI.h"

#include <algorithm>
#include <math.h>

#define BLOB_UNDEF 100000

FilterLinePointsROI::FilterLinePointsROI(int area):Filter(OF_ZERO, area, area){

	this->sc = SystemConfig::getInstance();

	MX = area/2;
	MY = area/2;

	Configuration *kh = (*this->sc)["KickHelper"];
	kickerCount = (int)kh->tryGet<int>(3, "KickConfiguration", "KickerCount", NULL);

	Configuration *loc = (*this->sc)["ROI"];
	LinePointsThreshold = (unsigned char)loc->get<int>("ROI", "LinePointsThreshold", NULL);
	LinePointsJump = (unsigned char)loc->get<int>("ROI", "LinePointsJump", NULL);
	MinLineWidth = (unsigned char)loc->get<int>("ROI", "MinLineWidth", NULL);
	MaxLineWidth = (unsigned char)loc->get<int>("ROI", "MaxLineWidth", NULL);

	kicker1.midX = (int)loc->get<int>("ROI", "Kicker1X", NULL);
	kicker1.midY = (int)loc->get<int>("ROI", "Kicker1Y", NULL);
	kicker1.left = kicker1.midX - 26;
	kicker1.right = kicker1.midX;
	kicker1.top = kicker1.midY - 28;
	kicker1.bottom = kicker1.midY + 28;


	kicker2.midX = (int)loc->get<int>("ROI", "Kicker2X", NULL);
	kicker2.midY = (int)loc->get<int>("ROI", "Kicker2Y", NULL);
	kicker2.left = kicker2.midX - 28;
	kicker2.right = kicker2.midX + 28;
	kicker2.top = kicker2.midY - 30;
	kicker2.bottom = kicker2.midY;

	kicker3.midX = (int)loc->get<int>("ROI", "Kicker3X", NULL);
	kicker3.midY = (int)loc->get<int>("ROI", "Kicker3Y", NULL);
	kicker3.left = kicker3.midX;
	kicker3.right = kicker3.midX + 26;
	kicker3.top = kicker3.midY - 28;
	kicker3.bottom = kicker3.midY + 28;


	init();

}


FilterLinePointsROI::FilterLinePointsROI(int width, int height):Filter(OF_ZERO, width, height){

	this->sc = SystemConfig::getInstance();

	Configuration *vision = (*this->sc)["Vision"];
	Configuration *loc = (*this->sc)["ROI"];

	MX = vision->get<int>("Vision", "CameraMX", NULL);
	MY = vision->get<int>("Vision", "CameraMY", NULL);

	LinePointsThreshold = (unsigned char)loc->get<int>("ROI", "LinePointsThresholdDirected", NULL);
	LinePointsJump = (unsigned char)loc->get<int>("ROI", "LinePointsJump", NULL);
	MinLineWidth = (unsigned char)loc->get<int>("ROI", "MinLineWidth", NULL);
	MaxLineWidth = (unsigned char)loc->get<int>("ROI", "MaxLineWidth", NULL);

	init();

}



FilterLinePointsROI::~FilterLinePointsROI(){

	cleanup();

}




inline short max(short a, short b) {
	return (a>b)? a:b;
}

inline short min(short a, short b) {
	return (a<b)? a:b;
}

inline unsigned char max(unsigned char a, short b) {
	return (a>b)? a:b;
}

inline unsigned char min(unsigned char a, unsigned char b) {
	return (a<b)? a:b;
}



std::vector<ROIData> FilterLinePointsROI::process(unsigned char * src, unsigned int width, unsigned int height, std::vector<LinePoint> & LinePoints, DistanceLookupHelper & distanceHelper, ScanLineHelperBall & helper){


	unsigned char * tgt = src;

	unsigned char * tmp = (unsigned char *) malloc(width * height);
	memcpy(tmp, src, width*height);

	double * LookupTable = distanceHelper.getLookupTable();

	short hist[256];
	short kumhist[256];
	memset(hist, 0, 256*sizeof(short));
	memset(kumhist, 0, 256*sizeof(short));

	short * lines = helper.getLines();
	int * lineOffsets = helper.getLineOffsets();

	short x;
	short y;


	LinePoints.clear();
	std::vector<short> LinePointsX;
	LinePointsX.clear();
	std::vector<short> LinePointsY;
	LinePointsY.clear();
	std::vector<short> LinePointsW;
	LinePointsW.clear();
	
	
	std::vector<short> LinePointsUnrecX;
	LinePointsUnrecX.clear();
	std::vector<short> LinePointsUnrecY;
	LinePointsUnrecY.clear();

	std::vector<unsigned char> roiMaxInt;
	LinePointsUnrecY.clear();



	for(short i = 0; i < helper.getNumberLines(); i++){

		short b = 0;
		short e = 0;

		short * line = lines + lineOffsets[i];
		short * linePtrBegin = line;

		short * lbegin = NULL;
		short * lend = NULL;
		short * lmin = lines + lineOffsets[i];
		short * lmax = lines + lineOffsets[i+1] - 2;

		x = *line++;
		y = *line++;

		short vb = src[x*width + y];

		short a = 1;

		for(int j = lineOffsets[i] + 2; j < lineOffsets[i+1]; j+=2){
			x = *line++;
			y = *line++;
			short va = src[x*width + y];

			if(va > LinePointsThreshold+20){
				short roilen=2;
				lbegin = line - 2;
				lend = line + 2;
				short maxInt=va;
				
				while(lend<lmax && 
				      src[(*lend)*width + (*(lend+1))] > LinePointsThreshold-30 &&
				      src[(*lend)*width + (*(lend+1))] > src[(*(lend+2))*width + (*(lend+3))] + LinePointsJump) {
					lend += 2;
					line += 2;
					j++;
					roilen++;
					maxInt = max(maxInt, src[(*lend)*width + (*(lend+1))]);
				}
				while(lbegin>lmin && 
				      src[(*lbegin)*width + (*(lbegin+1))] > LinePointsThreshold-30) {
				      //src[(*lbegin)*width + (*(lbegin+1))] > src[(*(lbegin+2))*width + (*(lbegin+3))] + LinePointsJump ) {
					lbegin -= 2;
					roilen++;
					maxInt = max(maxInt, src[(*lbegin)*width + (*(lbegin+1))]);
				}

				if((lend-lbegin)/2 > MinLineWidth && (lend-lbegin)/2 < MaxLineWidth) {
					short indX = *(lbegin+((roilen/2)*2));
					short indY = *(lbegin+((roilen/2)*2) + 1);
					LinePointsX.push_back(indY);
					LinePointsY.push_back(indX);
					LinePointsW.push_back(((lend-lbegin)/2 + 12)/2);
					hist[maxInt]++;
					roiMaxInt.push_back(maxInt);
				}
			}
			vb = va;
			a++;
		}

	
	}

	//calc kumulative histogram
	kumhist[0] = hist[0];
	for(int i=1; i<256; i++) {
		kumhist[i] = kumhist[i-1] + hist[i];
	}
	unsigned char minCol = ((float)kumhist[255])*0.9;
	unsigned char minInt;
	for(minInt=254; minInt>0; minInt--) {
		if(kumhist[minInt] < minCol) break;		
	}

	//find connected components
 	std::vector<ROIData> ROIrects;
	ROIrects.clear();
	ROIData rect;

	short left, right, top, bottom;
	for(int n=0; n<LinePointsX.size(); n++) {
		if(roiMaxInt[n]<minInt) continue;
		right  = LinePointsX[n]+LinePointsW[n];
		left   = LinePointsX[n]-LinePointsW[n];
		top    = LinePointsY[n]-LinePointsW[n];
		bottom = LinePointsY[n]+LinePointsW[n];

		//printf("LinePoint: %d %d %d %d\n", left,top, right,bottom);

		for(int m=n; m<LinePointsX.size(); m++) {
			if(roiMaxInt[n]<minInt) continue;
			if(right > LinePointsX[m] && left < LinePointsX[m] && top < LinePointsY[m] && bottom > LinePointsY[m]) {
				//one component? -> connect components together
				left = min(left, LinePointsX[m]-LinePointsW[m]);
				right = max(right, LinePointsX[m]+LinePointsW[m]);
				top = min(top, LinePointsY[m]-LinePointsW[m]);
				bottom = max(bottom, LinePointsY[m]+LinePointsW[m]);
			}
		}

		rect.left = max(left, 1);
		rect.right = min(right, width-2);
		rect.top = max(top, 1);
		rect.bottom = min(bottom, height-2);		

		rect.midX = (right+left)/2;
		rect.midY = (top+bottom)/2;

		bool valid = true;
		bool replace = false;

		//DASHIER IST EIN ECHT DRECKIGER HACK!!!
		//if(rect.bottom-rect.top > 100 || rect.right-rect.left>100) valid = false;
		//HACK ENDE

		for(int m=0; m<ROIrects.size(); m++) {
			if(ROIrects[m].left < rect.midX && ROIrects[m].right > rect.midX && ROIrects[m].bottom > rect.midY && ROIrects[m].top < rect.midY) {
			//RIO is a other ROI -> ignore
				valid = false;
			}
			if(ROIrects[m].midX > rect.left && ROIrects[m].midX < rect.right && ROIrects[m].midY > rect.bottom && ROIrects[m].midY < rect.top) {
				replace = true;
				valid = false;
				ROIrects[m] = rect;
			}
		}
		if(valid) ROIrects.push_back(rect);
	}
	if(kickerCount>1) ROIrects.push_back(kicker1);
	if(kickerCount>0) ROIrects.push_back(kicker2);
	if(kickerCount>2) ROIrects.push_back(kicker3);

	for(int m=ROIrects.size()-1; m>=0; m--) {
		if(ROIrects[m].bottom-ROIrects[m].top > 100 || ROIrects[m].right-ROIrects[m].left>100) {
			ROIrects.erase(ROIrects.begin()+m);
			m++;
		}

		//This should never happen!!!
		if((ROIrects[m].left < 0) || (ROIrects[m].top < 0) || (ROIrects[m].bottom >= height) || (ROIrects[m].right >= width)) {
			ROIrects.erase(ROIrects.begin()+m);
			m++;
		}
	}

//find ROI within a ROI
/*
	ROIData dat;
	for(int m=0; m<ROIrects.size(); m++) {
		dat = ROIrects[m];
		for(int y=dat.top; y<dat.bottom; y++) {
			unsigned char minV=255, maxV=0;
			int sum=0;
			int count=0;
			for(int x=dat.left; x<dat.right; x++) {
				if(src[x+y*width] < minV) minV = src[x+y*width];
				if(src[x+y*width] > maxV) maxV = src[x+y*width];
				sum += src[x+y*width];
				count++;
			}
			sum = sum/count;

			if(maxV < sum+40) {
				for(int x=dat.left; x<dat.right; x++) {
					if(src[x+y*width] < maxV-20) tgt[x+y*width] = 0;
				}
			}
		}
	}
*/

//ab hier wird nurnoch umkopiert.
	printf("FilterLinePointsROI - Number of LinePoints: %d %d\n", (int)LinePointsX.size(), (int)ROIrects.size());


// 	memset(tgt, 0, width*height);
// 
// 	for(unsigned int i = 0; i < LinePoints.size(); i++){
// 
// 		for(int a = -LinePointsW[i]/2; a <= LinePointsW[i]/2; a++){
// 			for(int b = -LinePointsW[i]/2; b <= LinePointsW[i]/2; b++){
// 				if(LinePointsX[i] + a >= 0 && LinePointsX[i] + a < height && LinePointsY[i] + b >= 0 && LinePointsY[i] + b < width)
// 					tgt[(LinePointsX[i] + a)*width + LinePointsY[i] + b] = tmp[(LinePointsX[i] + a)*width + LinePointsY[i] + b];
// 			}
// 		}
// 
// 	}
// 
// 	for(unsigned int i = 0; i < LinePoints.size(); i++){
// 
// 		for(int a = -2; a <= 2; a++){
// 			if(LinePointsX[i] + a >= 0 && LinePointsX[i] + a < height)
// 			tgt[(LinePointsX[i] + a)*width + LinePointsY[i]] = 0;
// 		}
// 
// 		for(int a = -2; a <= 2; a++){
// 			if(LinePointsY[i] + a >= 0 && LinePointsY[i] + a < width)
// 			tgt[(LinePointsX[i])*width + LinePointsY[i] + a] = 0;
// 		}
// 	}


// 	for(unsigned int i = 0; i < LinePointsUnrecX.size(); i++){
// 
// 		for(int a = -2; a <= 2; a++){
// 			if(LinePointsUnrecX[i] + a >= 0 && LinePointsUnrecX[i] + a < height)
// 			tgt[(LinePointsUnrecX[i] + a)*width + LinePointsUnrecY[i]] = 0;
// 		}
// 
// 	}
// 
// 	for(unsigned int i = 0; i < LinePointsUnrecX.size(); i++){
// 
// 		for(int a = -6; a <= 6; a++){
// 			for(int b = -6; b <= 6; b++){
// 				if(LinePointsUnrecX[i] + a >= 0 && LinePointsUnrecX[i] + a < height && LinePointsUnrecY[i] + b >= 0 && LinePointsUnrecY[i] + b < width)
// 				tgt[(LinePointsUnrecX[i] + a)*width + LinePointsUnrecY[i] + b] = tmp[(LinePointsUnrecX[i] + a)*width + LinePointsUnrecY[i] + b];
// 			}
// 		}
// 
// 	}



	free(tmp);

	return ROIrects;

}


void FilterLinePointsROI::visualizeROIs(unsigned char * src, std::vector<ROIData>& ROIrects, int width, int height) {
//Visalize ROI
	for(int m=0; m<ROIrects.size(); m++) {
		src[ROIrects[m].midX + (ROIrects[m].midY*width)] = 128;
		src[ROIrects[m].midX + (ROIrects[m].midY*width) + 1] = 128;
		src[ROIrects[m].midX + (ROIrects[m].midY*width) - 1] = 128;

		src[ROIrects[m].midX + (ROIrects[m].midY+1)*width] = 128;
		src[ROIrects[m].midX + (ROIrects[m].midY-1)*width] = 128;

		int l,r,t,b;
		l=ROIrects[m].left;
		r=ROIrects[m].right;
		t=ROIrects[m].top;
		b=ROIrects[m].bottom;
		for(int i=l; i<r; i++) {
			src[t*width+i] = 128;
			src[b*width+i] = 128;
		}

		for(int i=t; i<b; i++) {
			src[i*width+l] = 128;
			src[i*width+r] = 128;
		}
	}

}


void FilterLinePointsROI::init(){

}


void FilterLinePointsROI::cleanup(){


}



