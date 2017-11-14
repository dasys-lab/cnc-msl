#include "msl_ptgrey_camera/CamPropertySetter.h"

using std::cout;
using std::endl;
using std::cerr;

namespace msl_ptgrey_camera
{

	CamPropertySetter::CamPropertySetter()
	{
		this->sc = supplementary::SystemConfig::getInstance();
		this->cam = nullptr;
		this->gamma = 0;
		this->gain = 0;
		this->hue = 0;
		this->saturation = 0;
		this->shutter = 0;
		this->wb1 = 0;
		this->wb2 = 0;

	}

	CamPropertySetter::~CamPropertySetter()
	{
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

		this->gamma = (*this->sc)["Vision2"]->get<double>("Vision2.Camera.Gamma", NULL);
		this->gain = (*this->sc)["Vision2"]->get<double>("Vision2.Camera.Gain", NULL);
		this->hue = (*this->sc)["Vision2"]->get<double>("Vision2.Camera.Hue", NULL);
		this->saturation = (*this->sc)["Vision2"]->get<double>("Vision2.Camera.Saturation", NULL);
		this->shutter = (*this->sc)["Vision2"]->get<double>("Vision2.Camera.Shutter", NULL);
		this->wb1 = (*this->sc)["Vision2"]->get<double>("Vision2.Camera.WB1", NULL);
		this->wb2 = (*this->sc)["Vision2"]->get<double>("Vision2.Camera.WB2", NULL);
	}

	void CamPropertySetter::setProperty(FlyCapture2::PropertyType propType, double value)
	{
		if (!cam)
		{
			cout << "No camera present! " << endl;
			return;
		}


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
		this->readConfigValues();
		this->setProperty(FlyCapture2::PropertyType::GAMMA,this->gamma);
		this->setProperty(FlyCapture2::PropertyType::GAIN,this->gain);
		this->setProperty(FlyCapture2::PropertyType::HUE,this->hue);
		this->setProperty(FlyCapture2::PropertyType::SATURATION,this->saturation);
		this->setProperty(FlyCapture2::PropertyType::SHUTTER,this->shutter);
	}


} /* namespace msl_ptgrey_camera */
