/*
 * $Id: Control.cpp 2142 2007-04-15 10:49:00Z jewollen $
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
#include "BasisControl.h"


#include <stdlib.h>


BasisControl::BasisControl(double _minMV, double _maxMV, double _minDiffMV, double _maxDiffMV, double _m, double _b, double _tCV, double _startMV){
	minMV=_minMV;
	maxMV=_maxMV;
	minDiffMV=_minDiffMV;
	maxDiffMV=_maxDiffMV;
	tCV=_tCV;
	MV=_startMV;
}


BasisControl::BasisControl(string file, string confName){
	SystemConfig* sCon = SystemConfig::getInstance();
	Configuration *camParam = (*sCon)[file.c_str()];

	cout<<file.c_str()<<" - "<<confName.c_str()<<endl;
	minMV=camParam->get<double>(file.c_str(), confName.c_str(), "minMV", NULL);

	maxMV=camParam->get<double>(file.c_str(), confName.c_str(), "maxMV", NULL);
	minDiffMV=camParam->get<double>(file.c_str(), confName.c_str(), "minDiffMV", NULL);
	maxDiffMV=camParam->get<double>(file.c_str(), confName.c_str(), "maxDiffMV", NULL);
	MV=camParam->get<double>(file.c_str(), confName.c_str(), "startMV", NULL);
}

double BasisControl::protectComputeVariable(double newMV){
	if (abs(newMV)<minDiffMV){
		if (newMV>0){newMV=minDiffMV;}
		else{newMV=-minDiffMV;}
	}

	if (abs(newMV)>maxDiffMV){
		if (newMV>0){newMV=maxDiffMV;}
		else{newMV=-maxDiffMV;}
	}
	return newMV; 
}

double BasisControl::protectManipulateVariable(double _MV){
	_MV=std::min(_MV, maxMV);
	_MV=std::max(_MV, minMV);
	return _MV;
}


double BasisControl::getManipulateVariable(double measuredValue){
	double newMV=computeManipulateVariable(measuredValue);
	newMV=protectComputeVariable(newMV);
	MV=protectManipulateVariable(newMV+MV);
	return MV;
}

double BasisControl::testManipulateVariable(double measuredValue){
	double newMV=computeManipulateVariable(measuredValue);
	newMV=protectComputeVariable(newMV);
	newMV=protectManipulateVariable(newMV+MV);
	return newMV;
}

double BasisControl::isManupilateVariableAllowed(double measuredValue){
	double testMV=computeManipulateVariable(measuredValue);
	testMV=protectComputeVariable(testMV);
	testMV+=MV;
	if (testMV>maxMV){return testMV-maxMV;}
	if (testMV<minMV){return minMV-testMV;}
	return 0;
}


void BasisControl::addtCV(double addVal, string paramName, int max, int min){
	cout<<"3"<<endl;
	tCV+=addVal;
	cout<<"Parameter Adjusted ";
	if (tCV>max){
		tCV=max;
		cout<<"- Maximalwert überschritten -";
	}
	if (tCV<min){
		tCV=min;
		cout<<"- Minimalwert unterschritten -";
	}
	cout<<paramName<<" "<<tCV<<endl;
	cout<<"5"<<endl;
}



