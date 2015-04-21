
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
#include "Whitepoint.h"

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>


//#include "floatfann.h"
using namespace std;

Whitepoint* Whitepoint::theWhitepointInstance = NULL;

Whitepoint* Whitepoint::getInstance(int width, int height, camera::ImagingSource* cam){
	if(theWhitepointInstance == NULL){
		theWhitepointInstance = new Whitepoint(width, height,cam);
	}else{
		return NULL;
	}
	return theWhitepointInstance;
} 

Whitepoint::Whitepoint(int width, int height, camera::ImagingSource* cam, string area):BasisAutoParam(width/2, height, 4, cam){
	pcU = new PControl(CONF_DAT, "U");
	pcV = new PControl(CONF_DAT, "V");
	ra = new ReferenceArea(width/2, height, CONF_DAT, area.c_str());

	SystemConfig* sCon = SystemConfig::getInstance();
	Configuration *camParam = (*sCon)[CONF_DAT];
	pcU->setTargetControlValue(camParam->get<int>(CONF_DAT, "U", NULL));
	pcV->setTargetControlValue(camParam->get<int>(CONF_DAT, "V", NULL));
	camera::ImagingSource::white_balance_t iniWp;
	iniWp.bu=camParam->get<int>(CONF_DAT, "U", "startMV", NULL);
	iniWp.rv=camParam->get<int>(CONF_DAT, "V", "startMV", NULL);
	cout<<iniWp.bu<<iniWp.rv<<endl;
	if (nImage!=0){cam->setWhiteBalance(iniWp);}

}

Whitepoint::Whitepoint(int width, int height, camera::ImagingSource* cam):BasisAutoParam(width/2, height, 4, cam){
	pcU = new PControl(CONF_DAT, "U");
	pcV = new PControl(CONF_DAT, "V");
    ra = new ReferenceArea(width/2, height, CONF_DAT, CONF_RA_W);

	SystemConfig* sCon = SystemConfig::getInstance();
	Configuration *camParam = (*sCon)[CONF_DAT];
	pcU->setTargetControlValue(camParam->get<int>(CONF_DAT, "U", NULL));
	pcV->setTargetControlValue(camParam->get<int>(CONF_DAT, "V", NULL));
	camera::ImagingSource::white_balance_t iniWp;
	iniWp.bu=camParam->get<int>(CONF_DAT, "U", NULL);
	iniWp.rv=camParam->get<int>(CONF_DAT, "V", NULL);
	if (nImage!=0){cam->setWhiteBalance(iniWp);}

}


Whitepoint::~Whitepoint(){
	delete(ra);
	delete(pcU);
	delete(pcV);
}


void Whitepoint::process(unsigned char * scr, int counter) {
	if (!isNImage(counter)){return;}
	//U
	UBrightness=ra->createHistoBrightness(scr, 1, false);
	newWp.bu=(short)pcU->getManipulateVariable(UBrightness);
	cout<<"camParam - nBu: "<<newWp.bu<<" - UBrightness"<<UBrightness<<endl; 
	//V
	VBrightness=ra->createHistoBrightness(scr, 3, false);
	newWp.rv=(short)pcV->getManipulateVariable(VBrightness);
	cout<<"camParam - nRv: "<<newWp.rv<<" - VBrightness"<<VBrightness<<endl;
	cam->setWhiteBalance(newWp);
}

void Whitepoint::setNewParam(){
	cam->setWhiteBalance(newWp);
}
	
void Whitepoint::showRefFlaeche(unsigned char * scr){
	memcpy(showImage, scr, imageSize*4*sizeof(unsigned char));
	ra->createHistoBrightness(showImage, 1, true);
	ra->createHistoBrightness(showImage, 3, true);	
	showRGBScr(showImage);
}

void Whitepoint::testWhitepoint(unsigned char *scr, camera::ImagingSource::white_balance_t wp, int hell){
	//U
	UBrightness=ra->createHistoBrightness(scr, 1, true);
	//V
	VBrightness=ra->createHistoBrightness(scr, 3, true);

	int zeile[5] = { (int)wp.bu, (int)wp.rv, hell, (int)min((int)(UBrightness+0.5), 255), (int)min((int)(VBrightness+0.5), 255)};
	saveCSV("wp.csv", 5, zeile, true, 2);
}
