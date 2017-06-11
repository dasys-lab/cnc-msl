/*
 * CNImage.h
 *
 *  Created on: Jun 2, 2017
 *      Author:  Lisa Martmann
 */

#ifndef INCLUDE_MSL_PTGREY_CAMERA_CNIMAGE_H_
#define INCLUDE_MSL_PTGREY_CAMERA_CNIMAGE_H_

#include <stdio.h>
#include <cstdint>

namespace msl_ptgrey_camera
{
	enum ChannelType {
		UNDEF,
		C_1,
		C_3
	};
	class CNImage
	{
	public:
		CNImage(const unsigned int height,const unsigned int width,ChannelType channelType, unsigned char* imgData, unsigned int pixelBits);
		CNImage(const unsigned int height,const unsigned int width,ChannelType channelType, unsigned int pixelBits);
		CNImage(const unsigned int height,const unsigned int);
		virtual ~CNImage();
		unsigned int rows;
		unsigned int cols;
		unsigned int pixelBits;
		ChannelType channelType;
		unsigned char* imgData;

	private:
	};

} /* namespace msl_ptgrey_camera */

#endif /* INCLUDE_MSL_PTGREY_CAMERA_CNIMAGE_H_ */
