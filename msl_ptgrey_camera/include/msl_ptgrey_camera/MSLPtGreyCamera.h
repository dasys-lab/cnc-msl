#ifndef MSL_PTGREY_CAMERA_MSLPTGREYCAMERA_H_
#define MSL_PTGREY_CAMERA_MSLPTGREYCAMERA_H_
#include <flycapture/FlyCapture2.h>
#include <iostream>
#include <cstdint>
#include <cstdarg>
#include <sys/types.h>
#include "msl_ptgrey_camera/CNImage.h"
#include "msl_ptgrey_camera/CamPropertySetter.h"

namespace msl_ptgrey_camera
{
//	class CamPropertySetter;
	class MSLPtGreyCamera
	{
	public:
		MSLPtGreyCamera();
		virtual ~MSLPtGreyCamera();
		int init(int rows, int cols, ChannelType numChannels,int bitsPerPixel);
//		void PrintError(FlyCapture2::Error error);
		CNImage* getNextImage();
		void setGamma(double value);
		void setShutter(double value);
		void setGain(double value);
		void setHue(double value);
		void setSaturation(double value);
		void setWhiteBalance(double redChannel, double blueChannel);
//		void imgCallback(class FlyCapture2::Image* pImage, const void* pCallbackData);
		FlyCapture2::GigECamera* cam;
		FlyCapture2::Image* camImg;
		CamPropertySetter* cps;
		CNImage* img;

	};

} /* namespace msl_ptgrey_camera */

#endif /* MSL_PTGREY_CAMERA_MSLPTGREYCAMERA_H_ */
