/*
 * CNImage.cpp
 *
 *  Created on: Jun 2, 2017
 *      Author:  Lisa Martmann
 */
#include "msl_ptgrey_camera/CNImage.h"

using namespace std;
namespace msl_ptgrey_camera
{

	CNImage::CNImage(const unsigned int rows, const unsigned int cols, ChannelType channels, unsigned char* imgData,
						const unsigned int pixelBits)
	{
		this->rows = rows;
		this->cols = cols;
		this->pixelBits = pixelBits;
		this->channelType = channels;
		this->imgData = imgData;
	}

	CNImage::CNImage(const unsigned int rows, const unsigned int cols, ChannelType channels,
						const unsigned int pixelBits)
	{
		this->rows = rows;
		this->cols = cols;
		this->channelType = channels;
		this->pixelBits = pixelBits;
		this->imgData = nullptr;
	}

	CNImage::CNImage(const unsigned int rows, const unsigned int cols)
	{
		this->rows = rows;
		this->cols = cols;
		this->channelType = UNDEF;
		this->pixelBits = 0;
		this->imgData = nullptr;
	}

	CNImage::~CNImage()
	{
		// TODO Auto-generated destructor stub
	}

} // namespace msl_ptgrey_camera
