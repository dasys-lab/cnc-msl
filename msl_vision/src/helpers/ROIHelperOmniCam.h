/*
 * $Id: ROIHelperOmniCam.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef ROIHelperOmniCam_H
#define ROIHelperOmniCam_H

#include <stdlib.h>
#include <SystemConfig.h>
#include "../global/Types.h"
#include "../filters/FilterExtractBlobs.h"
#include "DistanceLookupHelper.h"

using namespace castor;

class ROIHelperOmniCam {

	public:

		ROIHelperOmniCam();
		~ROIHelperOmniCam();

		static ROIHelperOmniCam * getInstance();

		ROI getROIForObject(double x, double y, double z, double radius, DistanceLookupHelper & distanceHelper);

	private:

		static ROIHelperOmniCam * instance_;

		SystemConfigPtr sc;

		int scWIDTH;
		int scHEIGHT;

		short mx;
		short my;
		int CameraZ;

		void goForLine(short ax, short ay, short ex, short ey, double angle, DistanceLookupHelper & distanceHelper, int * xcoord, int * ycoord);

		void init();
		void cleanup();

};

#endif



