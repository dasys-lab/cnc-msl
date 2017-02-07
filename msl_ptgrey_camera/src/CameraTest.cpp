/*
 * CameraTest.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: cn
 */

#include "msl_ptgrey_camera/CameraTest.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <flycapture/FlyCapture2.h>
using namespace std;
using namespace cv;

namespace msl_ptgrey_camera
{

	CameraTest::CameraTest()
	{
		// TODO Auto-generated constructor stub

	}

	CameraTest::~CameraTest()
	{
		// TODO Auto-generated destructor stub
	}

} /* namespace msl_ptgrey_camera */

const int col_size   = 1280;
const int row_size   = 960;
const int data_size  = row_size * col_size;

void PrintError(FlyCapture2::Error error);

void imgCallback(class FlyCapture2::Image* pImage, const void* pCallbackData )
{
	cout << "callback!" << endl;
}

int main(int argc, char** argv)
{
	cout << "Test" << endl;
	namedWindow("MSLCameraPreview", WINDOW_NORMAL);
	//void* windowHandle = cv::ge("CameraImage");
	resizeWindow("MSLCameraPreview", 800, 600);

	FlyCapture2::GigECamera cam;
	FlyCapture2::CameraInfo camInfo;
	FlyCapture2::Error error;
	FlyCapture2::BusManager busMgr;
	FlyCapture2::PGRGuid guid;
	//Format7PacketInfo fmt7PacketInfo;
	//Format7ImageSettings fmt7ImageSettings;

	FlyCapture2::TriggerMode triggerMode;


	// Get Flea2 camera

	busMgr.ForceAllIPAddressesAutomatically();
	error = busMgr.GetCameraFromIndex( 0, &guid );

	if ( error != FlyCapture2::PGRERROR_OK )
	{
	PrintError( error );
		return -1;
	}

	// Connect to the camera
	error = cam.Connect( &guid );
	if ( error != FlyCapture2::PGRERROR_OK )
	{
		PrintError( error );
		return -1;
	}


	// Get camera information
	error = cam.GetCameraInfo(&camInfo);
	if ( error != FlyCapture2::PGRERROR_OK )
	{
		PrintError( error );
		return -1;
	}


	// Set Config
	FlyCapture2::FC2Config config;
	config.grabMode = FlyCapture2::GrabMode::DROP_FRAMES;
	cam.SetConfiguration(&config);

	// Set Imaging Mode
	cam.SetGigEImagingMode(FlyCapture2::MODE_0);


	FlyCapture2::GigEImageSettings imageSettings;
	imageSettings.pixelFormat = FlyCapture2::PixelFormat::PIXEL_FORMAT_RAW8;
	imageSettings.height = 960;
	imageSettings.width = 1280;
	cam.SetGigEImageSettings(&imageSettings);

	// Start Capture
	error = cam.StartCapture();
	if ( error != FlyCapture2::PGRERROR_OK )
	{
		PrintError( error );
		return -1;
	}

	FlyCapture2::Image camImg;

	//CvMemStorage* storage = NULL;
	IplImage* img    = NULL;
	img = cvCreateImage( cvSize( 1280, 960 ),
					 IPL_DEPTH_8U,
					 4 );

	int frame = 0;
	while(true) {
		//cam.FireSoftwareTrigger(true);


		cam.RetrieveBuffer(&camImg);


		cout << "frame: " << frame++ << " data size: " << camImg.GetDataSize() << endl;
		//cout << "data:" << camImg.GetData() << endl;


		// Create OpenCV structs for grayscale image


		memcpy(img->imageData, camImg.GetData(), camImg.GetDataSize());
		//cvSaveImage( "test.bmp", img );

		cvShowImage("MSLCameraPreview", img);

		cvWaitKey(0);

	}

	cvSaveImage( "test.bmp", img );

	return 0;
}


// Print error trace
void PrintError( FlyCapture2::Error error )
{
    error.PrintErrorTrace();
}
