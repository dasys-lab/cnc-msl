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
#include "BasisAutoParam.h"

#include <stdlib.h>

BasisAutoParam::BasisAutoParam(int _width, int _height, int _usedDim, camera::ImagingSource* _cam){
	xvDisplayGray=NULL;
	xvDisplayRGB=NULL;
	xvDisplayScr=NULL; 

	height=_height;
	width=_width;
	imageSize=_height*_width;
	usedDim=_usedDim;
	cam=_cam;

	cam->enableAutoShutter(false);
	cam->enableAutoGain(false);	
	cam->disableAutoWhiteBalance();
	cam->enableTrigger(false); 
	cam->enableGamma(false);

	SystemConfig* sCon = SystemConfig::getInstance();
	Configuration *camParam = (*sCon)[CONF_DAT];
	nImage=camParam->tryGet<int>(10, CONF_DAT, "nImage", NULL);
	showImage=(unsigned char*)malloc(imageSize*sizeof(unsigned char)*4);

}

BasisAutoParam::~BasisAutoParam(){
	if(xvDisplayGray != NULL)	delete(xvDisplayGray);
	if(xvDisplayRGB != NULL)		delete(xvDisplayRGB);
	if(xvDisplayScr != NULL)		delete(xvDisplayScr);
	if(showImage != NULL)		free(showImage);
}

bool BasisAutoParam::isNImage(int counter){
	if (nImage==0){return false;}
	if (counter%nImage==0){return true;}
	return false;
}

/*unsigned char* BasisAutoParam::copyImage(unsigned char* image, int _imageDim, int _width, int _height){
	if (_imageDim==-1){_imageDim=4;}
	if (_width==-1){_width=width;}
	if (_height==-1){_height=height;}
	int _imageSize=_width*_height*_imageDim;
	unsigned char *uePixel =(unsigned char*) malloc(_imageSize);
	if (image!=NULL){
		memcpy(uePixel, image, _imageSize);
	}else{
		memset(uePixel, STD_COLOR, _imageSize);
	}
	return uePixel;
}*/


void BasisAutoParam::showRGB(unsigned char *RGBImage, int _width, int _height, XVDisplay **display){
	if (_width==-1){_width=width;}
	if (_height==-1){_height=height;}
	if (display==NULL){display=&xvDisplayRGB;}

	if (*display==NULL){
		*display = new XVDisplay(_width, _height, XV_UYVY);
		(**display).displayFrameRGB((char*)RGBImage);
	}
	(**display).displayFrameRGB((char*)RGBImage);
}


void BasisAutoParam::showRGBScr(unsigned char *scr, int _width, int _height){
	if (_width==-1){_width=width*2;}
	if (_height==-1){_height=height;}
	FilterYUVToRGB filterYUVToRGB(_width, _height);
	unsigned char *RGBIma = filterYUVToRGB.process(scr, _width*_height*2);
	showRGB(RGBIma, _width, _height, &xvDisplayScr);
}

void BasisAutoParam::showGrayScr(unsigned char *scr, int _width, int _height){
	if (_width==-1){_width=width*2;}
	if (_height==-1){_height=height;}
	int _imageSize=_width*_height;
	unsigned char* grayImage=(unsigned char*)malloc(_imageSize);
	for (int i=0; i<_imageSize; i++){
		grayImage[i]=scr[i*2+1];
	}
	if (xvDisplayGray==NULL){
		xvDisplayGray = new XVDisplay(_width, _height, XV_UYVY);
		xvDisplayGray->displayFrameGRAY((char*)grayImage);
	} 
	xvDisplayGray->displayFrameGRAY((char*)grayImage);
	free(grayImage);
}

void BasisAutoParam::saveCSV(string pfad, int args, int *argv, bool print, int delByFirstZeros){
	for (int i=0; delByFirstZeros!=-1 && i<args; i++){
		if (argv[i]!=0){break;}
		if (i==delByFirstZeros-1){
			remove(pfad.c_str());
		}
	}
	fstream f;
	f.open(pfad.c_str(), ios::out | ios::app);
	for (int i=0; i<args; i++){
		f<<argv[i]<<",";
	}
	f<<"\n";
	f.close();

	for (int i=0; i<args && print; i++){
		cout<<argv[i]<<"\t";
	}
	if (print){cout<<endl;}
}






