/*
 * ImageHandler.h
 *
 *  Created on: Nov 24, 2010
 *      Author: elm
 */

#ifndef IMAGEHANDLER_H_
#define IMAGEHANDLER_H_

#include <Magick++.h>

//#include "/usr/include/ImageMagick/Magick++.h"
//#include "okto/MulticastSender.h"
//#include "okto/OktoImage.h"

//#include "elmstoolbox/StopWatch.h"
//#include "elmstoolbox/ElmUtils.h"
//#include "elmstoolbox/Thread.h"
//#include <boost/thread/thread.hpp>
#include <thread.hpp>
#include <fstream>
#include <iostream>
#include <string.h>

#include <sys/stat.h>
#include <stdio.h>
#include <error.h>
#include <sys/types.h>

class ImageHandler {
public:
	ImageHandler(int imageWidth, int imageHeight, int fragments);
	virtual ~ImageHandler();
	bool sendp(unsigned char* data, int size, int imageWidth, int imageHeight, int stepX, int stepY, int x, int y, char imageType);
	void writeImage(unsigned char* i, bool isGray, int sizeX, int sizeY);
	void initImageWriter();

	std::ofstream outFile;
//	okto::MulticastSender* ms;
//	network::MulticastSender* ms;

	unsigned char* image;
	unsigned char* imageRaw;
	long long imageRawTime;
	int imageWidth;
	int imageHeight;
	int fragments;
	Magick::Image* image2;
	int quality;
	bool sending;
	bool savingRaw;
	bool run;

	boost::thread* threadRaw;
	boost::thread* threadSend;


};

#endif /* IMAGEHANDLER_H_ */
