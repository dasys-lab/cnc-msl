
/*
 * $Id: FilterAutoParamBasis.h 2124 2007-04-14 17:01:41Z jewollen $
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
#ifndef FilterAutoParamBasiss_H
#define FilterAutoParamBasiss_H


#include "../filters/FilterYUVToRGB.h"
#include "../XVDisplay.h"
//#include </home/cluehr/cn/Spica/Castor/SystemConfig.h>

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


#define GRAY 1
#define RGB 2
#define STD_COLOR 100

#define CONF_DAT "VisControl"


using namespace std;
using namespace supplementary;


class BasisAutoParam{


	public:
		BasisAutoParam(int width, int height, int dim, camera::ImagingSource* _cam);
		~BasisAutoParam();

		camera::ImagingSource* cam;
	//protected:
		//unsigned char* copyImage(unsigned char* image, int dim, int _width=-1, int _height=-1);
		void showRGB(unsigned char *RGBImage, int _width=-1, int _height=-1, XVDisplay **display=NULL);
		void showRGBScr(unsigned char *scr, int _width=-1, int _height=-1);
		void showGrayScr(unsigned char *scr, int _width=-1, int _height=-1);

		void saveCSV(string pfad, int args, int *argv, bool print, int delByFirstZeros=-1);
		bool isNImage(int counter);

		int width, height, imageSize, usedDim;
		map <int, int>nextPrioPixel;
		map <int, int>pixelPrio;
		int falseEdgePixel;
		int *histogr;
		unsigned char *showImage; 
		int nImage;


	private:
		XVDisplay *xvDisplayGray;
		XVDisplay *xvDisplayRGB;
		XVDisplay *xvDisplayScr;
};



#endif
