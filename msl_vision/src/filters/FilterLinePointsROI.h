/*
 * $Id: FilterLinePointsROI.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef FilterLinePointsROI_H
#define FilterLinePointsROI_H

#include <vector>
#include "Filter.h"

#include "../helpers/DistanceLookupHelper.h"
#include "../helpers/ScanLineHelperBall.h"
#include "../helpers/LinePoint.h"
#include "../global/Types.h"

#include <SystemConfig.h>

using namespace supplementary;

class FilterLinePointsROI  : public Filter {


	public:
		FilterLinePointsROI(int area);
		FilterLinePointsROI(int width, int height);
		~FilterLinePointsROI();
		
		//unsigned char * process(unsigned char * src, unsigned int width, unsigned int height, std::vector<LinePoint> & LinePoints, DistanceLookupHelper & distanceHelper, ScanLineHelperBall & helper);

		std::vector<ROIData> process(unsigned char * src, unsigned int width, unsigned int height, std::vector<LinePoint> & LinePoints, DistanceLookupHelper & distanceHelper, ScanLineHelperBall & helper);

		void visualizeROIs(unsigned char * src, std::vector<ROIData>& ROIrects, int width, int height);
	protected:

		SystemConfig* sc;

		void init();
		void cleanup();

		int MX;
		int MY;
	
		ROIData kicker1, kicker2, kicker3;
		int kickerCount;

		unsigned char LinePointsThreshold;
		unsigned char LinePointsJump;
		unsigned char MinLineWidth;
		unsigned char MaxLineWidth;



};




#endif

