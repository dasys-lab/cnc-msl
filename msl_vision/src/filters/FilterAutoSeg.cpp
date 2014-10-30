/*
 * $Id: FilterYUVToRGB.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "FilterBitmapWriter.h"

#include <algorithm>
#include <string.h>

FilterBitmapWriter::FilterBitmapWriter(int width, int height):Filter(OF_ZERO, width, height) {

	init();

}


FilterBitmapWriter::~FilterBitmapWriter(){

	cleanup();

}
		

unsigned char* FilterBitmapWriter::process(FILE *fid, unsigned char * src, unsigned int width, unsigned int height){
	
/*
	BITMAPINFOHEADER infoHeader;
	BITMAPFILEHEADER fileHeader;

	fileHeader.bfType = 0x4D42;
	fileHeader.bfSize = width*height*3;
	fileHeader.bfOffBits = 40;

	infoHeader.biSize = 40;
	infoHeader.biWidth = width;
	infoHeader.biHeight = height;
	infoHeader.biPlanes = 3;
	infoHeader.biBitCount = 24;
	infoHeader.biCompression =0;
	infoHeader.biSizeImage = width*height*3;
	infoHeader.biXPelsPerMeter = 0;
	infoHeader.biYPelsPerMeter = 0;
	infoHeader.biClrUsed = 0;
	infoHeader.biClrImportant = 0;
*/

	PPMHEADER ppmHeader;
	ppmHeader.width = width;
	ppmHeader.height = height;
	ppmHeader.magicKey = "P6";
	ppmHeader.maxColors = 255;

	fprintf(fid,"%s %d %d %d ", ppmHeader.magicKey,ppmHeader.width,ppmHeader.height,ppmHeader.maxColors);
	
	
	//fwrite(&fileHeader, 1, sizeof(BITMAPFILEHEADER), fid);
	//fwrite(&infoHeader, 1,sizeof(BITMAPINFOHEADER),fid);

	fwrite(src,1, width*height*3,fid);
	
	unsigned char * tgt = outputBuffer;

    return outputBuffer;


}


void FilterBitmapWriter::init(){


}


void FilterBitmapWriter::cleanup(){

}

