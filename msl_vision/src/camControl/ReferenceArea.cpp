/*
 * $Id: FilterYUVExtractSubImages.cpp 2142 2007-04-15 10:49:00Z jewollen $
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
#include "ReferenceArea.h"

#include <stdlib.h>

ReferenceArea::ReferenceArea(int _width, int _height, string file, string confName){

	memset(brightness, 0, sizeof(double)*4);

	height=_height;
	width=_width;
	imageSize=_height*_width;

	setNextPrioRefPixels(file, confName);
}


ReferenceArea::~ReferenceArea(){
}



void ReferenceArea::setNextPrioRefPixels(string file, string confName){

	prioPixelSize=0;
	SystemConfig* sCon = SystemConfig::getInstance();
	Configuration *camParam = (*sCon)[file.c_str()];

    	nextPrioPixel.clear();
    	pixelPrio.clear();

	vector<rect> rects;// = new vector<rect>;
	vector<circle> circles;
	stringstream area;
	rect tempRect={0,0,0,0,0};
	circle tempCircle={0,0,0,0};

	//read config
	for (int i=1; true; i++){

		stringstream rectn;
		rectn<<"Rect"<<i;
		//DEBUG 
//tempRect.top=160; tempRect.bottom=110; tempRect.left=110; tempRect.right=160; tempRect.prio=1;
		tempRect.bottom=camParam->tryGet<int>(-1, file.c_str(), confName.c_str(), rectn.str().c_str(), "bottom", NULL);
		if (tempRect.bottom==-1){
			cout<<file.c_str()<<"."<<confName.c_str()<<"."<<rectn.str().c_str()<<"."<<"bottom"<<endl;
			break;
		}
		tempRect.top=camParam->get<int>(file.c_str(), confName.c_str(), rectn.str().c_str(), "top", NULL);
cout<<"hier2"<<endl;
		tempRect.right=camParam->get<int>(file.c_str(), confName.c_str(), rectn.str().c_str(), "right", NULL);
		tempRect.left=camParam->get<int>(file.c_str(), confName.c_str(), rectn.str().c_str(), "left", NULL);
		tempRect.prio=camParam->get<int>(file.c_str(), confName.c_str(), rectn.str().c_str(), "prio", NULL);
		rects.push_back(tempRect);
	}


	for (int i=1; true; i++){
		stringstream circlen;
		circlen<<"Circle"<<i;
		cout << circlen.str()<<endl;
		/*DEBUG*/  
		//tempCircle.mx=225; tempCircle.my=225; tempCircle.r=240; tempCircle.prio=0;
		tempCircle.r=camParam->tryGet<int>(-1, file.c_str(), confName.c_str(), circlen.str().c_str(), "r", NULL);
		if (tempCircle.r==-1){break;}
		tempCircle.mx=camParam->get<int>(file.c_str(), confName.c_str(), circlen.str().c_str(), "mx", NULL);
		tempCircle.my=camParam->get<int>(file.c_str(), confName.c_str(), circlen.str().c_str(), "my", NULL);
		tempCircle.prio=camParam->get<int>(file.c_str(), confName.c_str(), circlen.str().c_str(), "prio", NULL);
		circles.push_back(tempCircle);
	}

	if (rects.size()+circles.size()==0){
		
	}

	//set Pixelprio
	for (int x=0; x<width; x++){
		for (int y=0; y<height; y++){
			//rects
			for (unsigned int i=0; i<rects.size(); i++){
				rect temp=rects.at(i);

				if (y>=temp.bottom && y<=temp.top && x*2>=temp.left && x*2<=temp.right) {
					pixelPrio[x+y*width]+=temp.prio;
					
					
				}
			}

			//cycles
			for (unsigned int i=0; i<circles.size(); i++){
				circle temp=circles.at(i);
				if (temp.my+(sqrt(temp.r*temp.r-((temp.mx-x*2)*(temp.mx-x*2))))>=y &&
					temp.my-(sqrt(temp.r*temp.r-((temp.mx-x*2)*(temp.mx-x*2))))<=y) {
					pixelPrio[x+y*width]+=temp.prio;
				}
			}
		}
	}
	int lastPrioPixel=0;
	for(int i=0; i<imageSize; i++){
		if (pixelPrio[i]==0){
			pixelPrio.erase(i);
			continue;
		}else{
			prioPixelSize+=pixelPrio[i];
			nextPrioPixel[lastPrioPixel]=i;
			lastPrioPixel=i;
		}
	}

	cout<<"Size: "<<nextPrioPixel.size()<<endl;

	nextPrioPixel[lastPrioPixel]=imageSize;
	int unusedHistoPixelPercent=camParam->tryGet<int>(20, file.c_str(), "unusedHistoPixelPercent", NULL);
	
	falseEdgePixel=(prioPixelSize*unusedHistoPixelPercent)/100;
}

 
void ReferenceArea::testArea(unsigned char *scr, double *rueck){
	double hell=createHistoBrightness(scr, 2, true);
	hell+=createHistoBrightness(scr, 4, true);
	hell/=2;
	double U=createHistoBrightness(scr, 1, true);
	double V=createHistoBrightness(scr, 3, true);
	cout<<"Hell: "<<hell<<"\t - ";
	cout<<"U: "<<U<<"\t - ";
	cout<<"V: "<<V<<"\t - ";
	cout<<"Size: "<<pixelPrio.size()-1<<endl;
	if (rueck!=NULL){
		rueck[0]=hell;
		rueck[1]=U;
		rueck[2]=V;
	}
}


double ReferenceArea::createHistoBrightness(unsigned char *scr, int dimPixel, bool printPrioPixel){

	dimPixel=dimPixel-1;

	memset(histo, 0, sizeof(int)*256);

	//First px
	if (pixelPrio[0]!=0){
		histo[scr[dimPixel]]+=pixelPrio[0];
	}

	for (int i=nextPrioPixel[0]; i<imageSize; i=nextPrioPixel[i]){
		histo[scr[i*4+dimPixel]]+=pixelPrio[i];
		if (printPrioPixel){scr[i*4+dimPixel]=255;}
	}
	
	/*if (printPrioPixel){printf("Histogramm von Dimension: %i",dimPixel);}
	for (int i=0; i<255 && printPrioPixel; i++){
		printf("%i-%i: %i %i %i %i %i\n",i,i+5, histo[i*4+dimPixel], histo[(i+1)*4+dimPixel], histo[(i+2)*4+dimPixel], histo[(i+3)*4+dimPixel], histo[(i+4)*4+dimPixel]);
	}*/

	//Brightness
	int firstColor=0;
	int lastColor=255;
	int restFirst=falseEdgePixel;
	int restLast=falseEdgePixel;

	//To dark area
	for (firstColor=0; restFirst>0; firstColor++){
		restFirst-=histo[firstColor];
	}
	firstColor--;

	//To light area
	for (lastColor=255; restLast>0 && lastColor>firstColor; lastColor--){
		restLast-=histo[lastColor];
	}
	lastColor++;

	if (firstColor==(lastColor-1)){
		restFirst+=restLast;
		restLast=0;
	}

	//Calculate the mean value
	long addPixelValue=0;
	for(int i=firstColor+1; i<lastColor; i++){
		addPixelValue+=histo[i]*i;
	}
	double dAddPixelValue=addPixelValue;
	dAddPixelValue+=(double)firstColor*((double)-restFirst);
	dAddPixelValue+=(double)lastColor*((double)-restLast);

	brightness[dimPixel]=dAddPixelValue/(double)(prioPixelSize-2*falseEdgePixel);
	return brightness[dimPixel];
}

/*
double ReferenceArea::createHistoBrightness(unsigned char *scr, int dimPixel, bool printPrioPixel, int uMin, int uMax, int vMin, int vMax){

	dimPixel=dimPixel-1;

	memset(histo, 0, sizeof(int)*256);

	//First px
	if (pixelPrio[0]!=0){
		histo[scr[dimPixel]]+=pixelPrio[0];
	}

	for (int i=nextPrioPixel[0]; i<imageSize; i=nextPrioPixel[i]){
		if (scr[i*4]>uMin && scr[i*4]<uMax && scr[i*4+2]>vMin && scr[i*4+2]<vMax){
			histo[scr[i*4+dimPixel]]+=pixelPrio[i];
			if (printPrioPixel){scr[i*4+dimPixel]=255;}
		}
	}


	for (int i=0; printPrioPixel && i<256; i++){
		if (histo[i]!=0){
			//cout<<"Farbe: "<<i<<" - Pixelanzahl: "<<histogr[dimPixel*256+i]<<" - Dimensionsindex: "<<(dimPixel*256+i)<<endl;
		}
	}

	//Brightness
	int firstColor=0;
	int lastColor=255;
	int restFirst=falseEdgePixel;
	int restLast=falseEdgePixel;

	//To dark area
	for (firstColor=0; restFirst>0; firstColor++){
		restFirst-=histo[firstColor];
	}
	firstColor--;

	//To light area
	for (lastColor=255; restLast>0 && lastColor>firstColor; lastColor--){
		restLast-=histo[lastColor];
	}
	lastColor++;

	if (firstColor==(lastColor-1)){
		restFirst+=restLast;
		restLast=0;
	}

	//Calculate the mean value
	long addPixelValue=0;
	for(int i=firstColor+1; i<lastColor; i++){
		addPixelValue+=histo[i]*i;
	}
	double dAddPixelValue=addPixelValue;
	dAddPixelValue+=(double)firstColor*((double)-restFirst);
	dAddPixelValue+=(double)lastColor*((double)-restLast);

	brightness[dimPixel]=dAddPixelValue/(double)(prioPixelSize-2*falseEdgePixel);
	/*if (printPrioPixel){
		cout<<"camParam - Size: "<<nextPrioPixel.size()<<endl;
		cout<<"camParam - Histo["<<dimPixel<<"]= "<<brightness[dimPixel]<<endl;}*/
	//return brightness[dimPixel];
//}


