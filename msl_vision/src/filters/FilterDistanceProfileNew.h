/*
 * $Id: FilterDistanceProfileNew.h 1935 2007-03-19 19:50:12Z phbaer $
 *
 *
 * Copyright 2005-2007 Carpe Noctem, Distributed Systems Group,
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
#ifndef FilterDistanceProfileNew_H
#define FilterDistanceProfileNew_H

#include "Filter.h"
#include "../helpers/ScanLineHelper.h"
#include "../helpers/DistanceLookupHelper.h"
#include "../global/Types.h"

#include <SystemConfig.h>

//#include <libAnja/DatagramSocket.h>
//#include <libAnja/UnixSocket.h>
#include <string>
#include <vector>

#define HORIZON 20000.0
#define RANGE 5

#define NSECTORS 60 

struct Range {

	short start;
	short end;
	double minDistance;


};

using namespace supplementary;

class FilterDistanceProfileNew  : public Filter {


	public:
		FilterDistanceProfileNew(int width, int height);
		~FilterDistanceProfileNew();
		
		unsigned char * process(unsigned char * src, unsigned int width, unsigned int height, ScanLineHelper & helper, DistanceLookupHelper & distanceHelper);

		double * getProfile();

		double getThreshold();

	protected:

		SystemConfig* sc;

		void init();
		void cleanup();

		void calculateOpponentsFromRange(Range range, double * profile, short profileLength, double sectorAngle, std::vector<Point> & opponentPoints);
		void DrawCircle(int midX, int midY, int rad, unsigned char * space, int width, int height);

		double * profile;
		double * tmpProfile;
		
		double threshold_;
		
		int numberOfLines;
		short minObsDistance, obsThreshOffset;
		short negRanges[3][2];
		std::vector<Holder> addHolders;


};




#endif

