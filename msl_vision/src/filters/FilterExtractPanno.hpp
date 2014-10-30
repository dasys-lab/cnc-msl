/*
 * $Id: FilterExtractImages.h 2124 2007-04-14 17:01:41Z jewollen $
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
#ifndef FilterExtractPanno_H
#define FilterExtractPanno_H

#include <SystemConfig.h>
#include <stdio.h>

#include "../global/Types.h"
#include "../XVDisplay.h"

using namespace std;
using namespace castor;

class FilterExtractPanno
{
	public:
		FilterExtractPanno(ImageSize &innerSize, ImageSize &outerSize, ImageSize &pannoSize);
		~FilterExtractPanno();
		
		void process(unsigned char *&src, unsigned char *&inner, unsigned char *&outer, unsigned char *&panno);
		uint16_t * getInnerTrafo() 	{	return innerPannoTrafo;	};
		uint16_t * getOuterTrafo()	{	return outerPannoTrafo;	};
		
	protected:
		uint16_t width;
		uint16_t height;
		/* Sizes of panorama image */
		uint16_t pWidth;
		uint16_t pHeight;
		uint16_t pHeightInner;
		uint16_t pHeightOuter;
		
		SystemConfigPtr sc;
		
		uint16_t iRadiusStart;
		uint16_t iRadiusEnd;
		uint16_t oRadiusStart;
		uint16_t oRadiusEnd;
		uint16_t iRadiusOffset;
		
		unsigned char * pannoImage;
		unsigned char * innerPanno;
		uint16_t * innerPannoTrafo;
		unsigned char * outerPanno;
		uint16_t * outerPannoTrafo;
		
		XVDisplay *pannoDisplay;
};

#endif
