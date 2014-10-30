/*
 * $Id: Filter.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef Filter_H
#define Filter_H

#define OF_ZERO 0
#define OF_RGB 1
#define OF_GRAY 2
#define OF_YUV422 3
#define OF_IPP8u 4
#define OF_IPP16s 5
#define OF_YUV_FULL 6

#include <string.h>
#include <stdlib.h>

class Filter {

	public:
		Filter(int bufferFormat_, int width, int heigth);
		~Filter();
		unsigned char * getOutputBuffer();

	protected:
		
		unsigned char * outputBuffer;
		int bufferFormat;

};

#endif

