
/*
 * $Id: FilterYUVExtractSubImages.h 2124 2007-04-14 17:01:41Z jewollen $
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
#ifndef Whitepoint_H
#define Whitepoint_H


#include "BasisAutoParam.h"
#include "ReferenceArea.h"
#include "PControl.h"
#include <SystemConfig.h>
#define CONF_RA_W "RAWhitepoint"

class Whitepoint : public BasisAutoParam {

	public:
		static Whitepoint* getInstance(int width, int height, camera::ImagingSource* _cam);

		void process(unsigned char *scr, int counter=-1);
		void testWhitepoint(unsigned char *scr, camera::ImagingSource::white_balance_t wp, int hell);
		void setNewParam();

		double getLastUBrightness(){return ra->getArithmHistoBrightness(1);};
		double getLastVBrightness(){return ra->getArithmHistoBrightness(3);};
		camera::ImagingSource::white_balance_t getLastWP(){return wp;};
        camera::ImagingSource::white_balance_t getLastNewWP(){return newWp;};

        void showRefFlaeche(unsigned char* currImage);

    private:
		//Singelton
        static Whitepoint* theWhitepointInstance;
        Whitepoint(int width, int height, camera::ImagingSource* _cam);
        Whitepoint(int width, int height, camera::ImagingSource* cam, string area);
        ~Whitepoint();
		Whitepoint():BasisAutoParam(0, 0, 0, NULL){}
		Whitepoint(const Whitepoint&):BasisAutoParam(0, 0, 0, NULL){}
        Whitepoint & operator = (const Whitepoint&);

		ReferenceArea *ra;
		PControl *pcU;
		PControl *pcV;
		double UBrightness;
		double VBrightness;

		camera::ImagingSource::white_balance_t wp, newWp;

		XVDisplay *xvDisplayRGB;
};


#endif

