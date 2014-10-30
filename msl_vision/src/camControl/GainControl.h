/*
 * $Id: GainControl.h 2124 2007-04-14 17:01:41Z jewollen $
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
#ifndef GainControl_H
#define GainControl_H

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
//#include "/home/cluehr/cn/Spica/Castor/SystemConfig.h"
//#include "/home/cluehr/cn/Spica/Castor/Configuration.h"
#include "BasisControl.h"
#include <pthread.h>

using namespace std;


class GainControl : public BasisControl{

	public:
		GainControl(string file, string confName);
		~GainControl(){};
		double computeManipulateVariable(double measuredValue);
		void addBright(double addVal);

	private:
		double hellGainLUT[256];
		double LUTAdd;
};

#endif
