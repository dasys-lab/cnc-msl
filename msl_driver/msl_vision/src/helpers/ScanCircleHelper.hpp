/*
 * $Id: ScanCircleHelper.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef ScanCircleHelper_H
#define ScanCircleHelper_H

#include <SystemConfig.h>

#include "../global/Types.h"

using namespace castor;

class ScanCircleHelper
{
	public:
		ScanCircleHelper();
		~ScanCircleHelper();
		
		uint16_t * getCircles() const;
		uint32_t * getCircleOffsets() const;
		uint16_t getNumberCircles() const;
		uint16_t getMaxPoints() const;
		
	protected:
		
		SystemConfigPtr sc;
		
		void init();
		
		uint16_t centerX;
		uint16_t centerY;
		uint16_t iRadiusStart;
		uint16_t iRadiusEnd;
		uint16_t oRadiusStart;
		uint16_t oRadiusEnd;
		uint16_t circleOffset;
		uint16_t maxPoints;
		
		uint16_t * circles;
		uint32_t * circleOffsets;
		uint16_t numberOfCircles;
		
		uint16_t width;
		uint16_t height;
};

#endif

