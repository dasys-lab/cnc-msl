/*
 * $Id: GainControl.cpp 2142 2007-04-15 10:49:00Z jewollen $
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
#include "GainControl.h"

#include <stdlib.h>


GainControl::GainControl(string file, string confName):BasisControl(file, confName){	
	SystemConfigPtr sCon = SystemConfig::getInstance();
	Configuration *camParam = (*sCon)[file.c_str()];

	LUTAdd=camParam->get<double>(file.c_str(), "GainLUT", "LUTGainAdd", NULL);
	for (int i=0; i<256; i++){
		stringstream hgi;
		hgi<<"HGLUT"<<i;
		hellGainLUT[i]=camParam->get<double>(file.c_str(), "GainLUT", hgi.str().c_str(), NULL);
		if (hellGainLUT[i]==0){hellGainLUT[i]=1.0/LUTAdd;}
	}
}


double GainControl::computeManipulateVariable(double mV){
	int addGain=0;
	if (mV<tCV){
		while (mV+hellGainLUT[(int)(mV+0.5)]<tCV){
			mV+=hellGainLUT[(int)(mV+0.5)];
			addGain+=LUTAdd;
		}
		double rest=(tCV-mV)/hellGainLUT[(int)(mV+0.5)]*LUTAdd;
		addGain+=rest;
		return addGain;
	}
	//Case two
	while (mV-hellGainLUT[(int)(mV+0.5)]>tCV){
		mV-=hellGainLUT[(int)(mV+0.5)];
		addGain-=LUTAdd;
	}
	double rest=(tCV-mV)/hellGainLUT[(int)(mV+0.5)]*LUTAdd;
	addGain+=rest;
	
	return addGain;
}

void GainControl::addBright(double addVal){
	addtCV(addVal, "Gain (in VisControl.conf)", 255, 5);
}


