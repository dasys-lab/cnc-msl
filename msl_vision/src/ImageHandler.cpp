/*
 * ImageHandler.cpp
 *
 *  Created on: Nov 24, 2010
 *      Author: elm
 */

#include "ImageHandler.h"

ImageHandler::ImageHandler(int imageWidth, int imageHeight, int fragments) {
	this->imageWidth = imageWidth;
	this->imageHeight = imageHeight;
	this->fragments = fragments;
	quality = 90;

	this->image = new unsigned char[imageWidth * imageHeight * 3];
	this->imageRaw = new unsigned char[imageWidth * imageHeight * 3];

	//	Magick::Image image2(Magick::Geometry(imageWidth / fragments / 2, imageHeight / fragments / 2), "white");
	image2 = new Magick::Image(Magick::Geometry(imageWidth / fragments / 2, imageHeight / fragments / 2), "white");

	// Set the image type to TrueColor DirectClass representation.
	image2->type(Magick::TrueColorType);
	image2->quality(quality);
	//	image2.compressType(Magick::BZipCompression);
	image2->compressType(Magick::JPEGCompression);
	image2->magick("JPG");
	std::cout << "compressType " << image2->compressType() << std::endl;
	std::cout << "quality " << image2->quality() << std::endl;

	// Ensure that there is only one reference to underlying image
	// If this is not done, then image pixels will not be modified.
	image2->modifyImage();

	sending = false;
	savingRaw = false;

	threadRaw = 0;
	threadSend = 0;

	run = true;
	
	mkdir("/dev/shm/CamPlayerCpp", 0777);
}

ImageHandler::~ImageHandler() {
	std::cout << "[ImageHandler] ~ImageHandler()" << std::endl;

	if (threadRaw != 0) {
		threadRaw->join();
		delete threadRaw;
	}
	std::cout << "[ImageHandler] ~ImageHandler() threadRaw ok" << std::endl;

	if (threadSend != 0) {
		threadSend->join();
		delete threadSend;
	}
	std::cout << "[ImageHandler] ~ImageHandler() threadSend ok" << std::endl;

	if (outFile.is_open()) {
		outFile.flush();
		outFile.close();
	}
	std::cout << "[ImageHandler] ~ImageHandler() close file ok" << std::endl;

	delete[] image;
	delete[] imageRaw;
	delete image2;
	std::cout << "[ImageHandler] ~ImageHandler() delete ok" << std::endl;

	std::cout << "[ImageHandler] ~ImageHandler() end" << std::endl;

}



void ImageHandler::writeImage(unsigned char* i, bool isGray, int sizeX, int sizeY) {
		
	// Create base image
	Magick::Image image(Magick::Geometry(sizeX, sizeY), "black"); //"white"

	// Set the image type to TrueColor DirectClass representation.
	image.type(Magick::TrueColorType);
	image.quality(50);
	image.compressType(Magick::BZipCompression);

	std::cout << "compressType " << image.compressType() << std::endl;
	std::cout << "quality " << image.quality() << std::endl;

	// Ensure that there is only one reference to underlying image
	// If this is not done, then image pixels will not be modified.
	image.modifyImage();

	// Allocate pixel view
	Magick::Pixels view(image);
	unsigned char* tmp = i;
	Magick::PixelPacket *pixels = view.get(0, 0, sizeX, sizeY);
	if (isGray) {
		for (int i = 0; i < sizeX * sizeY; ++i) {
			pixels->red = (*tmp) * 256;
			pixels->green = (*tmp) * 256;
			pixels->blue = (*tmp++) * 256;
			pixels++;
		}
	} else {
		for (int i = 0; i < sizeX * sizeY; ++i) {
			pixels->red = (*tmp++) * 256;
			pixels->green = (*tmp++) * 256;
			pixels->blue = (*tmp++) * 256;
			pixels++;
		}
	}		

	view.sync();
	image.write("/dev/shm/CamPlayerCpp/screenshot.jpg");

}

void ImageHandler::initImageWriter() {
	char dateiname[256] = "";
	struct tm *lokalZeit;
	time_t Zeit;
	time(&Zeit);
	lokalZeit = localtime(&Zeit);
	sprintf(dateiname, "/elm/vision_%d%02d%02d-%02d%02d%02d.raw", lokalZeit->tm_year + 1900, lokalZeit->tm_mon + 1, lokalZeit->tm_mday, lokalZeit->tm_hour, lokalZeit->tm_min, lokalZeit->tm_sec);
	std::cout << "initImageWriter datum " << dateiname << "\n";
	outFile.open(dateiname);
	std::cout << "open initImageWriter ok\n";

}


