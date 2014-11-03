/*
 * $Id: Filter.cpp 2141 2007-04-15 09:31:23Z phbaer $
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
#include "Filter.h"

#include <stdlib.h>

Filter::Filter(int bufferFormat_, int width, int height){
	outputBuffer = NULL;

	bufferFormat = bufferFormat;
	unsigned int bufferSize = 0;

	switch (bufferFormat_){

		case OF_ZERO:
			bufferSize = 0;
			break;
		case OF_RGB:
			bufferSize = width*height*3;
			break;
		case OF_GRAY:
			bufferSize = width*height;
			break;
		case OF_YUV422:
			bufferSize = width*height*2;
			break;
		case OF_IPP8u:
			bufferSize = width*height;
			break;
		case OF_IPP16s:
			bufferSize = width*height*2;
			break;
		case OF_YUV_FULL:
			bufferSize = width*height*3;
			break;
		default:
			bufferSize = 0;
	

	}

	if (bufferSize > 0)	{
		outputBuffer = (unsigned char *) malloc(bufferSize);
	}
}



Filter::~Filter(){
	if (outputBuffer != NULL) {
		free(outputBuffer);
	}
}


unsigned char * Filter::getOutputBuffer(){

	return outputBuffer;

}

