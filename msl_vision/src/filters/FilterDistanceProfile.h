/*
 * $Id: FilterDistanceProfile.h 1935 2007-03-19 19:50:12Z phbaer $
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
#ifndef FilterDistanceProfile_H
#define FilterDistanceProfile_H

#include "Filter.h"
#include "../helpers/ScanLineHelper.h"
#include "../helpers/DistanceLookupHelper.h"

#include <SystemConfig.h>

//#include <libAnja/DatagramSocket.h>
//#include <libAnja/UnixSocket.h>
#include <string>

#define HORIZON 20000.0
#define RANGE 5

#define NSECTORS 60 

using namespace supplementary;

class FilterDistanceProfile  : public Filter {


	public:
		FilterDistanceProfile(int width, int height, bool _calibMode = false);
		~FilterDistanceProfile();
		
		unsigned char * process(unsigned char * src, unsigned int width, unsigned int height, unsigned char color, ScanLineHelper & helper, DistanceLookupHelper & distanceHelper, bool printOutput);

		short * calculateNewNegRanges();

		double * getProfile();

	protected:

		SystemConfig* sc;

		void init();
		void cleanup();

		double * profile;
		double * tmpProfile;
		
		bool calibMode;
		int * calibProfile;
		int calibCounter;

		int numberOfLines;

		//Anja::Socket * socket;
		//std::string destAddress;
		//std::string socketType;
		//int destPort;
		//unsigned short msgid;
		short negRanges[3][2];


};




#endif

