
//============================================================================
// Name        : ga.cpp
// Author      : Claas Lühring
// Version     : Eins
// Copyright   : 1000€
// Description : Misst das Aussehen einer Referenzfläche bei verschiedenen Gainparametern.
//============================================================================

#include <iostream>

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "../Brightness.h"
#include "../Whitepoint.h"
#include <Configuration.h>
#include "../../driver/imagingsource.h"

using namespace std;

int main() {

	camera::ImagingSource* cam = NULL;	
	
	//Je nach Kamera muss eine der folgenden Zeilen ein- bzw. auskommentiert sein.
	cam = new camera::ImagingSource("Point Grey");
	//cam = new camera::ImagingSource("Imaging Source");
	cam->setVideoMode(DC1394_VIDEO_MODE_640x480_YUV422);
	cam->init();

	//cam->setFramerate(DC1394_FRAMERATE_30);  //DC1394_FRAMERATE_15

	//Wird nicht gebraucht
	cam->enableAutoShutter(false);
	cam->enableTrigger(false);
	cam->enableGamma(false);
	cam->setGamma(false);
	cam->enableAutoGain(false);
	cam->disableAutoWhiteBalance();

	//Set Hue
	unsigned char ucHue = 180;
	cam->setHue(ucHue);
	std::cout << "Hue: "<< (int)ucHue << std::endl;

	//Set Expo
	unsigned short usExpo = 0;
	cam->setExposure(usExpo);
	std::cout << "Expo :"<<  usExpo << std::endl;

	//Set Saturation
	unsigned char ucSat = 127;
	cam->setSaturation(ucSat);
	std::cout << "Sat: "<< (int) ucSat << std::endl;


	cam->startCapture();
	camera::Frame frame;
	cam->getFrame(frame);

    Brightness *br = Brightness::getInstance(640, 480, cam, "RABrightness", 80);
//	ReferenceArea *rfbW=new ReferenceArea(320, 480, "VisControl", "RAHelligkeit");
		
	
	int counter=0;
	
	camera::ImagingSource::white_balance_t sWp;
	sWp.bu=780;
	sWp.rv=600;
	cam->setWhiteBalance(sWp);

	for (int s=1025; s<1226; s+=50){
		for (int g=10; g<800; g+=20){
			counter++;
			cam->setShutter(s);
			cam->setGain(g);
			for (int i=0; i<10; i++){
				cam->getFrame(frame);
			} 
			
			br->testHelligkeit(frame.getImagePtr(), g, s);
			br->showRefFlaeche(frame.getImagePtr());
		}
	}
	return 0;
}
