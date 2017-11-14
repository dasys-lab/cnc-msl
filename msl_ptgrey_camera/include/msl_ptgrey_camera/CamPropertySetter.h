#pragma once

#include "SystemConfig.h"
#include <FlyCapture2.h>



namespace msl_ptgrey_camera
{
	class CamPropertySetter
	{
	public:
		CamPropertySetter();
		virtual ~CamPropertySetter();
		void readConfigValues();
		void setCamera(FlyCapture2::GigECamera* cam);
		void setDefaults();
		void setProperty(FlyCapture2::PropertyType propType, double valueA, double valueB);
		void setProperty(FlyCapture2::PropertyType propType, double value);
	protected:
		supplementary::SystemConfig* sc;
		FlyCapture2::GigECamera* cam;
		double gamma;
	    double gain;
		double hue;
		double saturation;
		double shutter;
		double wb1;
		double wb2;
	private:
	};

} /* namespace msl_ptgrey_camera */

