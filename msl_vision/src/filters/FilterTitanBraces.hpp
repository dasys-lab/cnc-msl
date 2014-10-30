/*
 * $Id: FilterLinePoints.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef FilterTitanBraces_H
#define FilterTitanBraces_H

#include "../helpers/ScanCircleHelper.hpp"
#include "../helpers/LinePoint.h"
#include "../global/Types.h"

#include <SystemConfig.h>
#include <string>
#include <fstream>

using namespace std;
using namespace castor;

class FilterTitanBraces
{
	public:
		FilterTitanBraces(ScanCircleHelper helper);
		~FilterTitanBraces();
		
		void process(unsigned char* src);
		
	protected:		
		uint16_t width;
		uint16_t height;
		
		uint16_t centerX;
		uint16_t centerY;
		
		uint16_t * circles;
		uint32_t * circlesOffset;
		uint16_t maxPoints;
		uint16_t nCircles;
		
		uint8_t CirclePointsThreshold;
		uint8_t CirclePointsJump;
		uint8_t MinBracesWidth;
		uint8_t MaxBracesWidth;
		uint8_t floorBrightness;
		
		std::vector<double> CirclePoints1;
		std::vector<double> CirclePoints2;
		std::vector<double> CirclePoints3;
};


#endif

