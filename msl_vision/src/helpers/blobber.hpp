
#ifndef BLOBBER_H
#define BLOBBER_H

#include <stdlib.h>
#include <stdint.h>
#include <iostream>
#include <stdio.h>

#include "../global/Types.h"
#include "../XVDisplay.h"
//#include "../../gnuplot-cpp/gnuplot_i.hpp"
#include "Distance3DHelper.hpp"


class Blobber
{
public:
	Blobber(ImageSize innerSize, ImageSize outerSize);
	~Blobber();
	
	void process(unsigned char * panno, Distance3DHelper distance3DHelper);
	
private:
	uint16_t width;
	uint16_t height;
	uint16_t innerHeight;
	uint16_t outerHeight;
	
	uint16_t blobSize;
	unsigned char * blob;
	uint16_t * matches;
	
	uint16_t trash;
	uint16_t trashBorder;
	
	unsigned char * image3D;
	unsigned char * image3D2;
	
	XVDisplay *pannoDisplay;
	XVDisplay *display3D;
	
	std::vector<double> X,Y,Z;
	
// 	Gnuplot plot;
};


#endif