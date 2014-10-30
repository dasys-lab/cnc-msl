#include <iostream>

#include <time.h>
#include <sys/time.h>

// Camera files
//#include "guppy.h"
//#include "marlin.h"
//#include "pike.h"
#include "../imagingsource.h"

#include "tester/DelayTester.h"

#include "XVDisplay.h"

int main ( int argc, char *argv[] )
{
    XVDisplay *xvDisplay;
    xvDisplay = new XVDisplay(640, 480, XV_UYVY);

    // Create a camera
    std::cout << "Camera Settings:" << std::endl;
    std::cout << std::endl;
    camera::ImagingSource camera(0);
    camera.printCameraModell();



    //Set Video Mode
    std::cout << "Video Mode: 640x480 YUV 422" << std::endl;
    camera.setVideoMode(DC1394_VIDEO_MODE_640x480_YUV422); //DC1394_VIDEO_MODE_640x480_MONO8

    // Initialise camera
    camera.init();

    //Set Framerate
    std::cout << "Framerate: 30" << std::endl;
    camera.setFramerate(DC1394_FRAMERATE_30);  //DC1394_FRAMERATE_15

    // Resetting camera
    //std::cout << "Soft resetting camera (sleeping 5 seconds)" << std::endl;
    //camera.softReset();

    //Set Shutter
    unsigned short usShutVal = 30;
    std::cout << "Shutter: "<<  usShutVal << std::endl;
    //camera.enableAutoShutter(true, 40000 , 40000); //40000 no effect
    camera.setShutter(usShutVal);

    //Set Gain
    unsigned short usGainVal = 100;
    std::cout << "Gain: "<< usGainVal << std::endl;
    camera.setGain(usGainVal);
    //camera.enableAutoGain(true);

    //Set Hue
    unsigned char ucHue = 180;
    camera.setHue(ucHue);
	std::cout << "Hue: "<< (int)ucHue << std::endl;

	//Set Expo
	unsigned short usExpo = 800;
	camera.setExposure(usExpo);
	std::cout << "Expo :"<<  usExpo << std::endl;

	//Set Saturation
	unsigned char ucSat = 127;
	camera.setSaturation(ucSat);
	std::cout << "Sat: "<< (int) ucSat << std::endl;

	//Set Brightness
	//unsigned short usBright = 100;
	//camera.setBrightness(usBright);
	//std::cout << "Bright: "<< usBright << std::endl;

    //Set AutoWhiteBalance
    //std::cout << "AutoWhiteBalance: on" << std::endl;
    //camera.opAutoWhiteBalance();


    // Start capturing
    std::cout << std::endl << std::endl << "Start capturing" << std::endl;
    camera.startCapture();


    //Print Feature Overview (all libdc features)
    //camera.printFeatures();

    camera::Frame frame;

    //init camera test
    cameratest::DelayTester delayTest;

	char input = 0;

    while (input != 'q')
    {
        /// get an image
        if (!camera.getFrame(frame))
        {
            std::cerr << "Error while capturing image, aborting!" << std::endl;
            exit(1);
        }
        //cvShowImage("CameraWindow", &image);
          xvDisplay->displayFrameYUV((char*) frame.getImagePtr());
	
          //Start Test
          delayTest.startDelayTest(frame);


        // get keyboard input
        /*switch (input = cvWaitKey(5))
        {
            // Mirror image
            case 'm': camera.enableMirror(!camera.isMirror());
                      std::cout << "Mirroring is now " << (camera.isMirror() ? "enabled" : "disabled") << std::endl;
                      break;

            // Print camera features
            case 'f': camera.printFeatures();
                      break;

            // Save image to the current directory
            case 's': sprintf(cstring, "img%08d.png", frames);
                      cvSaveImage(cstring, &image);
                      break;

            // Enable or disable HDR
            case 'h': camera.enableHDR(!camera.isHDR(), 100, 200);
                      std::cout << "HDR is now " << (camera.isHDR() ? "enabled" : "disabled") << std::endl;
                      break;

            // Recalculate whiteimage balance
            case 'l': camera.opAutoWhiteBalance();
                      break;

            // Output frame count
            case 'c': std::cout << "Frame Count " << camera.getFrameCount() << std::endl;
                      break;

            case 'r': camera.softReset();
                      break;

            // Quit
            case 'q': camera.stopCapture();
                      cvDestroyWindow( "DC1394Control" );
                      break;
        }*/

    }

    return 0;
}

