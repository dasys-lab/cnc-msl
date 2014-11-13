/*
 * $Id: ReferenceArea.h 2124 2007-04-14 17:01:41Z jewollen $
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
#ifndef ReferenceArea_H
#define ReferenceArea_H

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
#include <pthread.h>
#include <SystemConfig.h> 

using namespace std;
using namespace supplementary;

class ReferenceArea{
 
	public:
		ReferenceArea(int width, int height, string _datei, string _confName);
		~ReferenceArea();

		double createHistoBrightness(unsigned char *scr, int dimPixel, bool printHistogr=false);
		//double createHistoBrightness(unsigned char *scr, int dimPixel, bool printHistogr=false, int uMin, int uMax, int vMin, int vMax);
		double getArithmHistoBrightness(int dim){return brightness[dim];};
	
		void testArea(unsigned char *scr, double *rueck=NULL);


	private:
		void setNextPrioRefPixels(string file, string confName);
		int width, height, imageSize;
		map <int, int>nextPrioPixel;
		int prioPixelSize;
		map <int, int>pixelPrio;
		int falseEdgePixel;
		int histo[256];
		double brightness[4];

		struct circle{
			int prio;
			int mx;
			int my;
			int r;
		};

		struct rect{
			int prio;
			int top;
			int right;
			int bottom;
			int left;
		};
};

#endif
