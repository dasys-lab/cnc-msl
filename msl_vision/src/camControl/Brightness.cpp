/*
 * $Id: FilterBrightness.cpp 2142 2007-04-15 10:49:00Z jewollen $
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
#include "Brightness.h"



//#include "../helpers/KeyHelper.h"


//#include "floatfann.h"
using namespace std;
//using namespace supplementary;

Brightness *Brightness::theBrightnessInstance = NULL;

Brightness *Brightness::getInstance(int width, int height, camera::ImagingSource *cam){
    if( theBrightnessInstance == NULL ){
		theBrightnessInstance = new Brightness(width, height,cam);
    }else{
        return NULL;
    }
	return theBrightnessInstance;
}

Brightness *Brightness::getInstance(int width, int height, camera::ImagingSource *cam, string area, int zielHell){
    if( theBrightnessInstance == NULL ){
        theBrightnessInstance = new Brightness(width, height,cam,area,zielHell);
    }else{
        return NULL;
    }
    return theBrightnessInstance;
}

//Test - Konstr
Brightness::Brightness(int width, int height, camera::ImagingSource *cam, string area, int zielHell):BasisAutoParam(width/2, height, 1, cam){

	ra = new ReferenceArea(width/2, height, CONF_DAT, area.c_str());
	gc = new GainControl(CONF_DAT, "Gain");
	sc = new PControl(CONF_DAT, "Shutter");

	cout<<"CONF_DAT"<<CONF_DAT<<endl;

	SystemConfig* sCon = SystemConfig::getInstance();
	Configuration *camParam = (*sCon)[CONF_DAT];

	gc->setTargetControlValue(zielHell);
	sc->setTargetControlValue(zielHell);
	if (nImage!=0){
		cam->setShutter(camParam->get<int>(CONF_DAT, "Shutter", "startMV", NULL));
		cam->setGain(camParam->get<int>(CONF_DAT, "Gain", "startMV", NULL));
	}
}


Brightness::Brightness(int width, int height, camera::ImagingSource* cam):BasisAutoParam(width/2, height, 1, cam){

    ra = new ReferenceArea(width/2, height, CONF_DAT, CONF_RA_B);
	gc = new GainControl(CONF_DAT, "Gain");
	sc = new PControl(CONF_DAT, "Shutter");

	cout<<"CONF_DAT"<<CONF_DAT<<endl;

	SystemConfig* sCon = SystemConfig::getInstance();
	Configuration *camParam = (*sCon)[CONF_DAT];

	gc->setTargetControlValue(camParam->get<int>(CONF_DAT, "Bright", NULL));
	sc->setTargetControlValue(camParam->get<int>(CONF_DAT, "Bright", NULL));
	if (nImage!=0){
		cam->setShutter(camParam->get<int>(CONF_DAT, "Shutter", "startMV", NULL));
		cam->setGain(camParam->get<int>(CONF_DAT, "Gain", "startMV", NULL));
	}
}

int Brightness::getGain()
{
    return gc->getLManipulateVariable();
}

int Brightness::getShutter()
{
    return shutter;
}

double Brightness::getBrightness()
{
    return brightness;
}

void Brightness::addBrightness(double addVal)
{
    gc->addBright(addVal);
}


Brightness::~Brightness(){ 
    delete(ra);
	delete(sc);
	delete(gc);
}


void Brightness::testHelligkeit(unsigned char *scr, int gain, int shutter){
	brightness=ra->createHistoBrightness(scr, 2, true);
	cout<<"Helligkeit: "<<brightness<<endl;
	int zeile[3] = { gain, shutter, (int)(brightness+0.5)};
	saveCSV("MW.csv", 3, zeile, true, 2);
}


void Brightness::setNewParam(){
	shutter=newShutter;
	gain=newGain;
	cam->setShutter(shutter);
	cam->setGain(gain);
}
		

void Brightness::process(unsigned char *scr, int counter){
	if (!isNImage(counter)){return;}
	int sollShutter=30;

	cout<<endl<<"camGain - Gain: "<<gc->getLManipulateVariable()<<" - Schu: "<<sc->getLManipulateVariable()<<endl;
	brightness=ra->createHistoBrightness(scr, 2, false);
	brightness+=ra->createHistoBrightness(scr, 4, false);
	brightness/=2;
	cout<<" - Y-Brightness: "<<brightness<<endl;
	double ngc=gc->isManupilateVariableAllowed(brightness);
	if (ngc!=0){
		cam->setShutter(sc->getManipulateVariable(brightness));
		//cout<<"camParam - Schu gestellt: "<<sc->getLManipulateVariable()<<endl;
	}else if (sc->getLManipulateVariable()!=sollShutter){
		cam->setShutter(sc->getManipulateVariable(brightness));
	}
	cam->setGain(gc->getManipulateVariable(brightness));
		
}

void Brightness::showRefFlaeche(unsigned char *scr){
	memcpy(showImage, scr, imageSize*4*sizeof(unsigned char));
	ra->createHistoBrightness(showImage, 2, true);	
	showRGBScr(showImage);
}

void Brightness::showRefFlaecheGray(unsigned char * scr, int counter){
	if (!isNImage(counter)){return;}
	memcpy(showImage, scr, imageSize*2*sizeof(unsigned char));
	ra->createHistoBrightness(showImage, 1, true);	
	showGrayScr(showImage);
}

	

