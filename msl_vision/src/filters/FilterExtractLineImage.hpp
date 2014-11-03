/*
 * $Id: FilterExtractLineImage.h 2124 2007-04-14 17:01:41Z jewollen $
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
#ifndef FilterExtractLineImage_H
#define FilterExtractLineImage_H

#include "../global/Types.h"

class FilterExtractLineImage
{
	public:
		FilterExtractLineImage();
		FilterExtractLineImage(struct ImageSize size);
		~FilterExtractLineImage();
		
		void process(unsigned char * &src, unsigned char * &lineImage_);
				
		void setLineLookupTableValue(int index, int value);
		int getLineLookupTableValue(int index);
		
	protected:
		void init();
		void cleanup();
		
		uint16_t width;
		uint16_t height;
		
		unsigned char * uvImage;
		unsigned char * grayImage;
		unsigned char * lineImage;
		
		unsigned char * lineLookupTable;
};

#endif
