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
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */
#include "FilterLinePointsROIDirected.h"

#include <algorithm>
#include <math.h>

#define BLOB_UNDEF 100000

FilterLinePointsROIDirected::FilterLinePointsROIDirected(int area):Filter(OF_ZERO, area, area){

	this->sc = SystemConfig::getInstance();

	MX = area/2;
	MY = area/2;

	Configuration *loc = (*this->sc)["ROI"];

	LinePointsThreshold = (unsigned char)loc->get<int>("ROI", "LinePointsThreshold", NULL);
	LinePointsJump = (unsigned char)loc->get<int>("ROI", "LinePointsJump", NULL);
	MinLineWidth = (unsigned char)loc->get<int>("ROI", "MinLineWidth", NULL);
	MaxLineWidth = (unsigned char)loc->get<int>("ROI", "MaxLineWidth", NULL);


	init();

}


FilterLinePointsROIDirected::FilterLinePointsROIDirected(int width, int height):Filter(OF_ZERO, width, height){

	this->sc = SystemConfig::getInstance();

	Configuration *vision = (*this->sc)["Vision"];
	Configuration *loc = (*this->sc)["ROI"];

	MX = vision->get<int>("Vision", "CameraMX", NULL);
	MY = vision->get<int>("Vision", "CameraMY", NULL);

	LinePointsThreshold = (unsigned char)loc->get<int>("ROI", "LinePointsThreshold", NULL);
	LinePointsJump = (unsigned char)loc->get<int>("ROI", "LinePointsJump", NULL);
	MinLineWidth = (unsigned char)loc->get<int>("ROI", "MinLineWidth", NULL);
	MaxLineWidth = (unsigned char)loc->get<int>("ROI", "MaxLineWidth", NULL);

	init();

}



FilterLinePointsROIDirected::~FilterLinePointsROIDirected(){

	cleanup();

}
		

// unsigned char * FilterLinePointsROIDirected::process(unsigned char * src, unsigned int width, unsigned int height, std::vector<LinePoint> & LinePoints, DistanceLookupHelper & distanceHelper, ScanLineHelperBall & helper){
// 
// 
// 	unsigned char * tgt = src;
// 
// 	unsigned char * tmp = (unsigned char *) malloc(width * height);
// 	memcpy(tmp, src, width*height);
// 
// 	double * LookupTable = distanceHelper.getLookupTable();
// 
// 	short * lines = helper.getLines();
// 	int * lineOffsets = helper.getLineOffsets();
// 
// 	short x;
// 	short y;
// 
// 
// 	LinePoints.clear();
// 	std::vector<short> LinePointsX;
// 	LinePointsX.clear();
// 	std::vector<short> LinePointsY;
// 	LinePointsY.clear();
// 	std::vector<short> LinePointsW;
// 	LinePointsW.clear();
// 	
// 	
// 	std::vector<short> LinePointsUnrecX;
// 	LinePointsUnrecX.clear();
// 	std::vector<short> LinePointsUnrecY;
// 	LinePointsUnrecY.clear();
// 
// 
// 	for(short i = 0; i < helper.getNumberLines(); i++){
// 
// 		short b = 0;
// 		short e = 0;
// 
// 		short * line = lines + lineOffsets[i];
// 		short * linePtrBegin = line;
// 
// 		x = *line++;
// 		y = *line++;
// 
// 		short vb = src[x*width + y];
// 
// 		short a = 1;
// 
// 		for(int j = lineOffsets[i]; j < lineOffsets[i+1]; j++){
// 			x = *line++;
// 			y = *line++;
// 			short va = src[x*width + y];
// 
// 			if(va > 160){
// 				
// 				LinePointsUnrecX.push_back(x);
// 				LinePointsUnrecY.push_back(y);
// 			}
// 
// 			if(va > vb + LinePointsJump && va > LinePointsThreshold){
// 				b = a;
// 				e = a;
// 			}
// 
// 			if(va < vb - LinePointsJump && va < 180){
// 				e = a;
// 			}
// 
// 			if(b > 0 & e-b > MinLineWidth & e-b < 20*MaxLineWidth){
// 
// 				short indX = linePtrBegin[((e+b)/2)*2];
// 				short indY = linePtrBegin[((e+b)/2)*2 + 1];
// 
// 				double angle = -atan2(1.0*indY - MY, 1.0*indX - MX);
// 				double dist = LookupTable[indX*width + indY];
// 				double roiThres = LinePointsThreshold + 0.5*(e-b);
// 				if(roiThres > 200.0)
// 					roiThres = 200.0;
// 
// 				if(((dist > 100.0 && (double) (e-b) > 8.0 - dist/1000.0) || dist < 0.0) && src[indX*width + indY] > 0.0){
// 
// 					//tgt[indX*width + indY] = 0;
// 					LinePointsX.push_back(indX);
// 					LinePointsY.push_back(indY);
// 					LinePointsW.push_back(e-b + 12);
// 					LinePoint p;
// 					p.x = dist*cos(angle);
// 					p.y = dist*sin(angle);
// 					
// 					LinePoints.push_back(p);
// 				}
// 
// 				//printf("LinePoint: %d %d %d\n", i, b, e);
// 				//printf("LinePointAngle: %f\n", angle/M_PI * 180);
// 				//printf("LinePointIndex: %d %d\n", indX - height/2, indY - width/2);
// 
// 				b = 0;
// 				e = 0;
// 
// 
// 
// 			}
// 			vb = va;
// 			a++;
// 		}
// 
// 	
// 	}
// 
// 	printf("FilterLinePoints - Number of LinePoints: %d\n", (int)LinePoints.size());
// 
// 
// 	//memset(tgt, 0, width*height);
// 
// 	for(unsigned int i = 0; i < LinePoints.size(); i++){
// 
// 		for(int a = -LinePointsW[i]/2; a <= LinePointsW[i]/2; a++){
// 			for(int b = -LinePointsW[i]/2; b <= LinePointsW[i]/2; b++){
// 				tgt[(LinePointsX[i] + a)*width + LinePointsY[i] + b] = tmp[(LinePointsX[i] + a)*width + LinePointsY[i] + b];
// 			}
// 		}
// 
// 	}
// 
// 	for(unsigned int i = 0; i < LinePoints.size(); i++){
// 
// 		for(int a = -2; a <= 2; a++){
// 			tgt[(LinePointsX[i] + a)*width + LinePointsY[i]] = 0;
// 		}
// 
// 		for(int a = -2; a <= 2; a++){
// 			tgt[(LinePointsX[i])*width + LinePointsY[i] + a] = 0;
// 		}
// 	}
// 
// 
// /*	for(unsigned int i = 0; i < LinePointsUnrecX.size(); i++){
// 
// 		for(int a = -2; a <= 2; a++){
// 			tgt[(LinePointsUnrecX[i] + a)*width + LinePointsUnrecY[i]] = 0;
// 		}
// 
// 	}
// 
// 	for(unsigned int i = 0; i < LinePointsUnrecX.size(); i++){
// 
// 		for(int a = -6; a <= 6; a++){
// 			for(int b = -6; b <= 6; b++){
// 				tgt[(LinePointsUnrecX[i] + a)*width + LinePointsUnrecY[i] + b] = tmp[(LinePointsUnrecX[i] + a)*width + LinePointsUnrecY[i] + b];
// 			}
// 		}
// 
// 	}*/
// 
// 
// 
// 	free(tmp);
// 
// 	return tgt;
// 
// }



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



std::vector<ROIData> FilterLinePointsROIDirected::process(unsigned char * src, unsigned int width, unsigned int height, std::vector<LinePoint> & LinePoints, DistanceLookupHelper & distanceHelper, ScanLineHelperDirected & helper){


	unsigned char * tgt = src;

//	unsigned char * tmp = (unsigned char *) malloc(150000);
	//double * LookupTable = distanceHelper.getLookupTable();

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
		
		for(int j = lineOffsets[i]; j < lineOffsets[i+1]; j++){
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
					//short indX = *(lbegin+((lend-lbegin)/2));
					//short indY = *(lbegin+((lend-lbegin)/2) + 1);
					short indX = *(lbegin+((roilen/2)*2));
					short indY = *(lbegin+((roilen/2)*2) + 1);
					LinePointsX.push_back(indY);
					LinePointsY.push_back(indX);
					LinePointsW.push_back(((lend-lbegin)/2 + 12)/2);
					hist[maxInt]++;
					roiMaxInt.push_back(maxInt);
				}
			}
/*
			if(va > vb + LinePointsJump && va > LinePointsThreshold){
				b = a;
				e = a;
			}

			if(va < vb - LinePointsJump && va < 180){
				e = a;
			}

			if(b > 0 & e-b > MinLineWidth & e-b < 20*MaxLineWidth){
				LinePoint p;
				short indX = linePtrBegin[((e+b)/2)*2];
				short indY = linePtrBegin[((e+b)/2)*2 + 1];

				double angle = -atan2(1.0*indY - MY, 1.0*indX - MX);
				double dist = LookupTable[indX*width + indY];
				double roiThres = LinePointsThreshold + 0.5*(e-b);
				if(roiThres > 200.0)
					roiThres = 200.0;

				if(((dist > 100.0 && (double) (e-b) > 8.0 - dist/1000.0) || dist < 0.0) && src[indX*width + indY] > 0.0){

					//tgt[indX*width + indY] = 0;
					LinePointsX.push_back(indY);
					LinePointsY.push_back(indX);
					LinePointsW.push_back((e-b + 12)/2);
					
					p.x = dist*cos(angle);
					p.y = dist*sin(angle);
					
					LinePoints.push_back(p);
				}

				//printf("LinePoint: %d %d %d\n", i, b, e);
				//printf("LinePoint: %d %d\n", indX, indY);
				//printf("LinePointAngle: %f\n", angle/M_PI * 180);
				//printf("LinePointIndex: %d %d\n", indX - height/2, indY - width/2);

				b = 0;
				e = 0;



			}*/
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

		for(int m=0; m<LinePointsX.size(); m++) {
			if(right > LinePointsX[m] && left < LinePointsX[m] && top < LinePointsY[m] && bottom > LinePointsY[m]) {
				//one component? -> connect components together
				left = min(left, LinePointsX[m]-LinePointsW[m]);
				right = max(right, LinePointsX[m]+LinePointsW[m]);
				top = min(top, LinePointsY[m]-LinePointsW[m]);
				bottom = max(bottom, LinePointsY[m]+LinePointsW[m]);
			}
		}

		rect.left = max(left, 0);
		rect.right = min(right, width);
		rect.top = max(top, 0);
		rect.bottom = min(bottom, height);
		
		rect.midX = (right+left)/2;
		rect.midY = (top+bottom)/2;

		bool valid = true;
		bool replace = false;
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
//Visalize ROI
	for(int m=0; m<ROIrects.size(); m++) {
		if(ROIrects[m].midX<0) ROIrects[m].midX=0;
		if(ROIrects[m].midY<0) ROIrects[m].midY=0;
		if(ROIrects[m].left<0) ROIrects[m].left=0;
		if(ROIrects[m].top<0) ROIrects[m].top=0;
		if(ROIrects[m].right<0) ROIrects[m].right=0;
		if(ROIrects[m].bottom<0) ROIrects[m].bottom=0;

		if(ROIrects[m].midX>width) ROIrects[m].midX=width;
                if(ROIrects[m].midY>height) ROIrects[m].midY=height;
                if(ROIrects[m].left>width) ROIrects[m].left=width;
                if(ROIrects[m].top>height) ROIrects[m].top=height;
                if(ROIrects[m].right>width) ROIrects[m].right=width;
                if(ROIrects[m].bottom>height) ROIrects[m].bottom=height;


		tgt[ROIrects[m].midX + (ROIrects[m].midY*width)] = 0;
		tgt[ROIrects[m].midX + (ROIrects[m].midY*width) + 1] = 0;
		tgt[ROIrects[m].midX + (ROIrects[m].midY*width) - 1] = 0;

		tgt[ROIrects[m].midX + (ROIrects[m].midY+1)*width] = 0;
		tgt[ROIrects[m].midX + (ROIrects[m].midY-1)*width] = 0;

		int l,r,t,b;
		l=ROIrects[m].left;
		r=ROIrects[m].right;
		t=ROIrects[m].top;
		b=ROIrects[m].bottom;
		for(int i=l; i<r; i++) {
			tgt[t*width+i] = 0;
			tgt[b*width+i] = 0;
		}

		for(int i=t; i<b; i++) {
			tgt[i*width+l] = 0;
			tgt[i*width+r] = 0;
		}
	}

//ab hier wird nurnoch umkopiert.
	printf("FilterLinePoints - Number of LinePoints: %d %d\n", (int)LinePointsX.size(), (int)ROIrects.size());


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



	//free(tmp);

	return ROIrects;

}



void FilterLinePointsROIDirected::visualizeROIs(unsigned char * src, std::vector<ROIData>& ROIrects, int width, int height) {
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




void FilterLinePointsROIDirected::init(){


}


void FilterLinePointsROIDirected::cleanup(){


}



