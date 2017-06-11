#include "msl_ptgrey_camera/MSLPtGreyCamera.h"

using namespace std;

namespace msl_ptgrey_camera
{

	MSLPtGreyCamera::MSLPtGreyCamera()
	{
		this->cam = nullptr;
		this->camImg = nullptr;
		this->cps = nullptr;
		this->img = nullptr;
	}

	MSLPtGreyCamera::~MSLPtGreyCamera()
	{
		// TODO Auto-generated destructor stub
		delete this->cam;
		delete this->camImg;
		delete this->cps;
		delete this->img;
	}

	void PrintError(FlyCapture2::Error error);

	void imgCallback(class FlyCapture2::Image* pImage, const void* pCallbackData)
	{
		cout << "callback!" << endl;
	}

	int MSLPtGreyCamera::init(int rows, int cols, ChannelType numChannels,int bitsPerPixel)
	{


		cam = new FlyCapture2::GigECamera();
		camImg = new FlyCapture2::Image();
		cps = new msl_ptgrey_camera::CamPropertySetter();
		img = new CNImage(rows, cols,numChannels, bitsPerPixel);
//		FlyCapture2::GigECamera cam;
		FlyCapture2::CameraInfo camInfo;
		FlyCapture2::Error error;
		FlyCapture2::BusManager busMgr;
		FlyCapture2::PGRGuid guid;
		//Format7PacketInfo fmt7PacketInfo;
		//Format7ImageSettings fmt7ImageSettings;


		FlyCapture2::TriggerMode triggerMode;
		// Get Flea2 camera

		busMgr.ForceAllIPAddressesAutomatically();
		error = busMgr.GetCameraFromIndex(0, &guid);

		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}
		// Connect to the camera
		error = cam->Connect(&guid);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

		// Get camera information
		error = cam->GetCameraInfo(&camInfo);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

		// Set Config
		FlyCapture2::FC2Config config;
		config.grabMode = FlyCapture2::GrabMode::DROP_FRAMES;
		cam->SetConfiguration(&config);

		// Set Imaging Mode
		cam->SetGigEImagingMode(FlyCapture2::MODE_0);

		FlyCapture2::GigEImageSettings imageSettings;
		imageSettings.pixelFormat = FlyCapture2::PixelFormat::PIXEL_FORMAT_RAW8;
		imageSettings.height = rows;
		imageSettings.width = cols;
		cam->SetGigEImageSettings(&imageSettings);

		// Start Capture
		error = cam->StartCapture();
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

		cps->setCamera(this->cam);
//		cps->setDefaults();

		return 0;

	}

	void MSLPtGreyCamera::setGamma(double value) {
		cps->setGamma(value);
	}

	void MSLPtGreyCamera::setShutter(double value) {
		cps->setShutter(value);
	}

	void MSLPtGreyCamera::setGain(double value) {
		cps->setGain(value);
	}

	void MSLPtGreyCamera::setHue(double value) {
		cps->setHue(value);
	}

	void MSLPtGreyCamera::setSaturation(double value) {
		cps->setSaturation(value);
	}

	void MSLPtGreyCamera::setWhiteBalance(double redChannel, double blueChannel) {
		cps->setWhiteBalance(redChannel,blueChannel);
	}

	void PrintError(FlyCapture2::Error error)
	{
		error.PrintErrorTrace();
	}

	CNImage* MSLPtGreyCamera::getNextImage()
	{
		cam->RetrieveBuffer(camImg);
		img->imgData = camImg->GetData();
		return img;
	}



} /* namespace msl_ptgrey_camera */
