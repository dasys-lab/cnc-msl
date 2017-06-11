/*
 * CamPropertySetter.h
 *
 *  Created on: May 16, 2017
 *      Author:  Lisa Martmann
 */

//#ifndef SRC_CAMPROPERTYSETTER_H_
//#define SRC_CAMPROPERTYSETTER_H_
#ifndef MSL_PTGREY_CAMERA_CAMPROPERTYSETTER_H_
#define MSL_PTGREY_CAMERA_CAMPROPERTYSETTER_H_

#include "SystemConfig.h"
#include <FlyCapture2.h>

using std::cout;
using std::endl;
using std::cerr;

namespace msl_ptgrey_camera
{
	class CamPropertySetter
	{
	public:
//		CamPropertySetter(FlyCapture2::GigECamera* cam);
		CamPropertySetter();
		virtual ~CamPropertySetter();
		void readConfigValues();
		void setCamera(FlyCapture2::GigECamera* cam);
		void setDefaults();
		void setProperty(FlyCapture2::PropertyType propType, double valueA, double valueB);
		void setProperty(FlyCapture2::PropertyType propType, double value);
		void setGamma(double value);
		void setGain(double value);
		void setHue(double value);
		void setSaturation(double value);
		void setShutter(double value);
		void setWhiteBalance(double redChannel, double blueChannel);
	protected:
		supplementary::SystemConfig* sc;
		FlyCapture2::GigECamera* cam;
		int gamma;
		int gain;
		int hue;
		int saturation;
		int shutter;
		int wb1;
		int wb2;
	private:
	};

} /* namespace msl_ptgrey_camera */

#endif /* MSL_PTGREY_CAMERA_CAMPROPERTYSETTER_H_ */
