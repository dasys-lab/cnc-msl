/*
 * CameraTest.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: cn
 */

#include "msl_ptgrey_camera/CameraTest.h"
using std::endl;
using std::cout;
using namespace cv;

CameraTest::CameraTest()
{
	// TODO Auto-generated constructor stub

}

CameraTest::~CameraTest()
{
	// TODO Auto-generated destructor stub
}

void onGammaTrackbarMoved(int value, void* camera)
{
	msl_ptgrey_camera::MSLPtGreyCamera* cam = (msl_ptgrey_camera::MSLPtGreyCamera*)camera;
	cam->setGamma((double)value / 10.0);
}

void onShutterTrackbarMoved(int value, void* camera)
{
	msl_ptgrey_camera::MSLPtGreyCamera* cam = (msl_ptgrey_camera::MSLPtGreyCamera*)camera;
	cam->setShutter((double)value / 100);

}

//Gain ranges from -4.46915 to 24.0001
void onGainTrackbarMoved(int value, void* camera)
{
	msl_ptgrey_camera::MSLPtGreyCamera* cam = (msl_ptgrey_camera::MSLPtGreyCamera*)camera;
	cam->setGain((double)value / 10 - 4.47);
}

//Saturation ranges from 0 to 399.902
void onSaturationTrackbarMoved(int value, void* camera)
{
	msl_ptgrey_camera::MSLPtGreyCamera* cam = (msl_ptgrey_camera::MSLPtGreyCamera*)camera;
	cam->setSaturation((double)value);
}

//Hue ranges from -180 to 179.912
void onHueTrackbarMoved(int value, void* camera)
{
	msl_ptgrey_camera::MSLPtGreyCamera* cam = (msl_ptgrey_camera::MSLPtGreyCamera*)camera;
	cam->setHue((double)value - 180);
}

void onWB1TrackbarMoved(int value, void* camera)
{
	msl_ptgrey_camera::MSLPtGreyCamera* cam = (msl_ptgrey_camera::MSLPtGreyCamera*)camera;
//	cam->setWhiteBalance()
}

void onWB2TrackbarMoved(int value, void* camera)
{
	msl_ptgrey_camera::MSLPtGreyCamera* cam = (msl_ptgrey_camera::MSLPtGreyCamera*)camera;
}
const cv::String WINDOWNAME = "MSLCameraPreview";

int main(int argc, char** argv)
{

	//Create and init PtGrey cam
	msl_ptgrey_camera::MSLPtGreyCamera cam;
	cam.init(1280, 960, msl_ptgrey_camera::ChannelType::C_3, 12);

	//Initialize starting values for Slider positions (min values for now)
	//TODO use system config
	int gammaSlider = 5;
	int shutterSlider = 3;
	int gainSlider = 0;
	int saturationSlider = 0;
	int hueSlider = 5;
	int wb1Slider = 5;
	int wb2Slider = 5;

	//Create window, trackbars and buttons
	namedWindow(WINDOWNAME, WINDOW_NORMAL);
	resizeWindow(WINDOWNAME, 800, 600);
	createTrackbar(cv::String("Gamma"), WINDOWNAME, &gammaSlider, 39, onGammaTrackbarMoved, &cam);
	createTrackbar(cv::String("Shutter"), WINDOWNAME, &shutterSlider, 3100, onShutterTrackbarMoved, &cam);
	createTrackbar(cv::String("Gain"), WINDOWNAME, &gainSlider, 240, onGainTrackbarMoved, &cam);
	createTrackbar(cv::String("Saturation"), WINDOWNAME, &saturationSlider, 4000, onSaturationTrackbarMoved, &cam);
	createTrackbar(cv::String("Hue"), WINDOWNAME, &hueSlider, 4000, onHueTrackbarMoved, &cam);
	createTrackbar(cv::String("WB1"), WINDOWNAME, &wb1Slider, 100, onWB1TrackbarMoved, &cam);
	createTrackbar(cv::String("WB2"), WINDOWNAME, &wb2Slider, 100, onWB2TrackbarMoved, &cam);

	Mat img(Size(1280, 960), CV_8UC1);


	int frame = 0;

	while (true)
	{

		cam.getNextImage();

		cout << "frame: " << frame++ << " data size: " << cam.camImg->GetDataSize() << endl;

		memcpy(img.data, cam.camImg->GetData(), cam.camImg->GetDataSize());

		Mat dest(Size(1280, 960), CV_8UC3);

		cvtColor(img, dest, CV_BayerGB2RGB);

		imshow("MSLCameraPreview", dest);
		cvWaitKey(0);

	}

	return 0;
}
