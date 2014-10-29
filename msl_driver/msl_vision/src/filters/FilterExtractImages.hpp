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
#ifndef FilterExtractImages_H
#define FilterExtractImages_H

#include <stdint.h>

class FilterExtractImages
{
	public:
		FilterExtractImages();
		~FilterExtractImages();
		
		void process(unsigned char * &src, unsigned char * &gray_image_, unsigned char * &uv_image_);
		
		bool setLookupTableValue(uint32_t const index, unsigned char const value);
		bool setLookupTableUVYValue(uint32_t const index, unsigned char const value);
		
		unsigned char getLookupTableValue(uint32_t const index) const;
		unsigned char getLookupTableUVYValue(uint32_t const index) const;
	protected:
		
		void init();
		
		uint16_t width;
		uint16_t height;
		
		uint16_t iRadiusStart;
		uint16_t iRadiusEnd;
		uint16_t oRadiusStart;
		uint16_t oRadiusEnd;
		uint16_t iRadiusOffset;
// 		
		unsigned char * uvImage;
		unsigned char * grayImage;
// 		
		uint32_t lTsize;
		uint32_t lTUVYsize;
		unsigned char *lookupTable;
		unsigned char * lookupTableUVY;
};

#endif
