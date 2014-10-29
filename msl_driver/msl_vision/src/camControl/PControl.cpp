/*
 * $Id: PControl.cpp 2142 2007-04-15 10:49:00Z jewollen $
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

#include "PControl.h"

#include <stdlib.h>

PControl::PControl(string file, string confName):BasisControl(file, confName){
	SystemConfigPtr sCon = SystemConfig::getInstance();
	Configuration *camParam = (*sCon)[file.c_str()];
	m=camParam->get<double>(file.c_str(), confName.c_str(), "m", NULL);
b=camParam->get<double>(file.c_str(), confName.c_str(), "b", NULL);	
}

double PControl::computeManipulateVariable(double measuredValue){
	double ret=(tCV-measuredValue)*m;
	if (tCV-measuredValue>0){ret+=b;}
	else {ret-=b;}
	return protectComputeVariable(ret);
}

