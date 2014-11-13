/*
 * $Id: FilterBrightness.h 2124 2007-04-14 17:01:41Z jewollen $
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
#ifndef Brightness_H
#define Brightness_H

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>

#include "BasisAutoParam.h"
#include "ReferenceArea.h"
#include "GainControl.h"
#include "PControl.h"
//#include <SystemConfig.h>

#define CONF_RA_B "RABrightness"

using namespace std;

class Brightness : public BasisAutoParam{

	public:
        static Brightness *getInstance(int width, int height, camera::ImagingSource *_cam);
        static Brightness *getInstance(int width, int height, camera::ImagingSource *_cam, string area, int zielHell);
		
		void process(unsigned char *scr, int counter=-1);
		void testHelligkeit(unsigned char *scr, int gain, int shutter);
		void setNewParam();
        int getGain();
        int getShutter();
        double getBrightness();

        void showRefFlaeche(unsigned char * scr);
        void showRefFlaecheGray(unsigned char * scr, int counter);

        void addBrightness(double addVal);

	private:
		//Singelton
        static Brightness* theBrightnessInstance;
        Brightness(int width, int height, camera::ImagingSource *_cam);
        Brightness(int width, int height, camera::ImagingSource *_cam, string area, int zielHell);
        ~Brightness();
		Brightness():BasisAutoParam(0, 0, 0, NULL){}
		Brightness(const Brightness&):BasisAutoParam(0, 0, 0, NULL){}
        Brightness & operator = (const Brightness&);

		void computeCamParam();

		ReferenceArea *ra;
		BasisControl *sc;
		GainControl *gc;

		double brightness;
		int tBrigtness, tShutter;
		int gain, shutter, newGain, newShutter;
		int minGain, maxGain;
		int tRegion, tRegionMinAdd;

		int LUTGainAdd;
		char hellGainLUT[256];
};

#endif
