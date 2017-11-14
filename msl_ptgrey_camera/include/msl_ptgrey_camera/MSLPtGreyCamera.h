#pragma once
#include <flycapture/FlyCapture2.h>
#include <iostream>
#include <cstdint>
#include <cstdarg>
#include <string>
#include <sys/types.h>
#include <SystemConfig.h>
#include "msl_ptgrey_camera/CamPropertySetter.h"

namespace msl_ptgrey_camera
{
	class MSLPtGreyCamera
	{
	public:
		MSLPtGreyCamera();
		virtual ~MSLPtGreyCamera();
		int init();

        unsigned char* getNextImage() const;

        void setGamma(double value);
		void setShutter(double value);
		void setGain(double value);
		void setHue(double value);
		void setSaturation(double value);
		void setWhiteBalance(double redChannel, double blueChannel);
        void saveCurrentImageToFile(string fpath);
//		void imgCallback(class FlyCapture2::Image* pImage, const void* pCallbackData);
		FlyCapture2::GigECamera* cam;
		FlyCapture2::Image* camImg;
		CamPropertySetter* cps;

    private:
        int imgHeight;
        int imgWidth;
        supplementary::SystemConfig* sc;
        void initialiseParameters();

	};

} /* namespace msl_ptgrey_camera */
