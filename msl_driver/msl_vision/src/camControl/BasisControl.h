/*
 * $Id: Control.h 2124 2007-04-14 17:01:41Z jewollen $
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
#ifndef Control_H
#define Control_H

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <sstream>
#include <map>
#include "../driver/imagingsource.h"
#include <SystemConfig.h>
#include <pthread.h>

using namespace std;
using namespace castor;
using namespace boost;;


class BasisControl{

	public: 
		BasisControl(double min, double max, double minDiff, double maxDiff, double m, double b, double targetControlValue, double startManipulationVariable);
		BasisControl(string file, string confName);
		~BasisControl(){};

		double getManipulateVariable(double measuredValue);
		double testManipulateVariable(double measuredValue);
		double getLManipulateVariable(){return MV;};
		double isManupilateVariableAllowed(double measuredValue);

		double getTargetControlValue(){return tCV;};

		void setTargetControlValue(double _tCV){tCV=_tCV;};

		


	protected:
		void addtCV(double addVal, string paramName, int max, int min);
		double protectManipulateVariable(double newMV);
		double protectComputeVariable(double CV);
		virtual double computeManipulateVariable(double measuredValue)=0;

		double MV;
		double minMV, maxMV, minDiffMV, maxDiffMV;
		double tCV;
};

#endif
