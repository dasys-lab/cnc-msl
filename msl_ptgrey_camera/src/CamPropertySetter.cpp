/*
 * CamPropertySetter.cpp
 *
 *  Created on: May 16, 2017
 *      Author:  Lisa Martmann
 */

#include "msl_ptgrey_camera/CamPropertySetter.h"

namespace msl_ptgrey_camera
{

	CamPropertySetter::CamPropertySetter()
	{
		// TODO Auto-generated constructor stub
		sc = supplementary::SystemConfig::getInstance();
		this->cam = nullptr;
		gamma = 0;
		gain = 0;
		hue = 0;
		saturation = 0;
		shutter = 0;
		wb1 = 0;
		wb2 = 0;

	}

	CamPropertySetter::~CamPropertySetter()
	{
		// TODO Auto-generated destructor stub
	}

	void CamPropertySetter::setCamera(FlyCapture2::GigECamera* camera)
	{
		if (!camera)
		{
			cout << "No camera present! " << endl;
			return;
		}
		this->cam = camera;
	}

	void CamPropertySetter::readConfigValues()
	{

		this->gamma = (*this->sc)["Vision"]->get<int>("Vision.Camera1394Settings.Gamma", NULL);
		this->gain = (*this->sc)["Vision"]->get<int>("Vision.Camera1394Settings.Gain", NULL);
		this->hue = (*this->sc)["Vision"]->get<int>("Vision.Camera1394Settings.Hue", NULL);
		this->saturation = (*this->sc)["Vision"]->get<int>("Vision.Camera1394Settings.Saturation", NULL);
		this->shutter = (*this->sc)["Vision"]->get<int>("Vision.Camera1394Settings.Shutter", NULL);
		this->wb1 = (*this->sc)["Vision"]->get<int>("Vision.Camera1394Settings.WB1", NULL);
		this->wb2 = (*this->sc)["Vision"]->get<int>("Vision.Camera1394Settings.WB2", NULL);
	}

	void CamPropertySetter::setProperty(FlyCapture2::PropertyType propType, double value)
	{
		if (!cam)
		{
			cout << "No camera present! " << endl;
			return;
		}

		//TODO wb special case?

		FlyCapture2::Property prop(propType);
//		FlyCapture2::Property prop;
		FlyCapture2::CameraInfo info;

		FlyCapture2::PropertyInfo inf(propType);
//		FlyCapture2::PropertyInfo inf;
		FlyCapture2::Error error1 = cam->GetPropertyInfo(&inf);

		cout << inf.absMin << ", " << inf.absMax << ", " << inf.absValSupported << ", " << inf.onOffSupported << ", " << inf.manualSupported << ", "
				<< inf.autoSupported << ", " << inf.onePushSupported << ", " << inf.readOutSupported << endl;

		if (error1 != FlyCapture2::PGRERROR_OK)
		{
			cout << "error while getting property info" << endl;
		}

		if (!inf.present)
		{
			cout << "PropertyInfo is not present!" << endl;
			return;
		}

		//TODO need?
//		if (inf.absValSupported)
//		{
//			prop.absControl = true;
//		}
//		else
//		{
//			cout << "AbsControl not supported!" << endl;
//			return;
//		}


		if (value > inf.absMax)
		{
			value = inf.absMax;
		}

		if (value < inf.absMin)
		{
			value = inf.absMin;
		}

		prop.autoManualMode = false;
		prop.onOff = true;
		prop.absControl = true;
		cout << "Value2: " << value << endl;
		prop.absValue = value;
		prop.valueA = value;
		prop.valueB = value;

		//TODO comment in ;)
		FlyCapture2::Error error = cam->SetProperty(&prop);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			cerr << "Property couldn't be set! " << endl;
			error.PrintErrorTrace();
		}

	}
	void CamPropertySetter::setProperty(FlyCapture2::PropertyType propType, double valueA, double valueB)
	{

		if (!cam)
		{
			cout << "No camera present! " << endl;
			return;
		}

		//TODO wb special case?

		FlyCapture2::PropertyInfo inf(propType);
		FlyCapture2::Property prop(propType);

//		if (!prop.present)
//		{
//			cout << "Property to set is not present!" << endl;
//			return;
//		}

		if (inf.absValSupported)
		{
			prop.absControl = true;
		}
		else
		{
			cout << "AbsControl not supported!" << endl;
			return;
		}
		prop.autoManualMode = false;

		if (valueA > inf.max)
		{
			valueA = inf.max;
		}
		if (valueB > inf.max)
		{
			valueB = inf.max;
		}

		if (valueA < inf.min)
		{
			valueA = inf.min;
		}

		if (valueB < inf.min)
		{
			valueB = inf.min;
		}

		prop.valueA = valueA;
		prop.valueB = valueB;

		FlyCapture2::Error error = cam->SetProperty(&prop);

		//which one? shouldn't be 2nd one
		if (error == FlyCapture2::PGRERROR_PROPERTY_FAILED || error == FlyCapture2::PGRERROR_PROPERTY_NOT_PRESENT)
		{
			cerr << "Property couldn't be set! " << endl;
			error.PrintErrorTrace();
			return;
		}

	}

	void CamPropertySetter::setDefaults()
	{
//		readConfigValues();
//		setGamma(this->gamma);
//		setGain(this->gain);
//		setHue(this->hue);
//		setSaturation(this->saturation);
//		setShutter(this->shutter);
	}

	void CamPropertySetter::setGamma(double value)
	{
		setProperty(FlyCapture2::PropertyType::GAMMA, value);
	}

	//Gain ranges from -4.46915 to 24.0001
	void CamPropertySetter::setGain(double value)
	{
		setProperty(FlyCapture2::PropertyType::GAIN, value);
	}

	void CamPropertySetter::setHue(double value)
	{
		setProperty(FlyCapture2::PropertyType::HUE, value);
	}

	void CamPropertySetter::setSaturation(double value)
	{
		setProperty(FlyCapture2::PropertyType::SATURATION, value);
	}

	//shutter ranges from 0.0299886 to 31.5883
	void CamPropertySetter::setShutter(double value)
	{
		setProperty(FlyCapture2::PropertyType::SHUTTER, value);
	}

	void CamPropertySetter::setWhiteBalance(double redChannel, double blueChannel)
	{
		setProperty(FlyCapture2::PropertyType::WHITE_BALANCE, redChannel, blueChannel);

	}

} /* namespace msl_ptgrey_camera */
