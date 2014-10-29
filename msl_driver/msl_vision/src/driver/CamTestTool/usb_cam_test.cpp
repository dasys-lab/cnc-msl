#include <iostream>

#include <time.h>
#include <sys/time.h>

// Camera files
//#include "guppy.h"
//#include "marlin.h"
//#include "pike.h"
//#include "../imagingsource.h"
#include "../CameraQuickCam.h"
#include "../ps3eye.h"

#include "FilterYUVToRGB.h"
#include "FilterYUVQuickCamToYUV.h"

//#include "tester/DelayTester.h"

#include "XVDisplay.h"

int main ( int argc, char *argv[] )
{
  
      int fps = 0;
      struct timeval tempo1, tempo2;
  
    		unsigned char * currImage = NULL;
		unsigned char * imageRGB = NULL;
		unsigned char * imageYUV = NULL;
		
		
    XVDisplay *xvDisplay;
    xvDisplay = new XVDisplay(640, 480, XV_UYVY);

    // Create a camera
    std::cout << "Create Camera" << std::endl;
    std::cout << std::endl;
    
    CameraQuickCam * camera = NULL;
    camera = new CameraQuickCam();

//     camera::Ps3Eye camera(0);
    std::cout << "init device" << std::endl;
//     camera.init();
    
    

    FilterYUVToRGB filterYUVToRGB(640, 480);
    FilterYUVQuickCamToYUV filterYUVQuickCamToYUV(640, 480);
    
    //needed?
//     			camera->set_auto_exposure_off();
// 			camera->set_sharpness(225);
// 			camera->set_brightness(201); 
			camera->set_contrast(70); 			
// 			camera->set_gamma(72);
// 			camera->set_saturation(128);
// 			camera->set_hue(0);
// 			camera->set_exposure(137);
// 
// 
// 			camera->set_gain(10);
    
//     camera.setGain(0);

//     std::cout << "start Capture" << std::endl;
//     camera.startCapture();
//     std::cout << "create frame structure" << std::endl;
//     camera::Frame frame;

	char input = 0;
	
// 	camera.setGain(0);

    while (input != 'q')
    {
    
      /*
      tempo2 = tempo1;
      gettimeofday(&tempo1, NULL);
      long long time = (tempo1.tv_sec - tempo2.tv_sec) * 1000000 + tempo1.tv_usec - tempo2.tv_usec;
      fps = 1000000 / time ;
      std::cout<<"fps " << time << " " << fps << "\n";
      std::cout << "start capture" << std::endl;
      */
      
      camera->captureBegin();
//       std::cout << "start capture" << std::endl;
//       if (!camera.getFrame(frame))
//         {
// // 	    std::cout << "test" << std::endl;
//             std::cerr << "Error while capturing image, aborting!" << std::endl;
//             exit(1);
//         }

//       std::cout << "get buffer" << std::endl;
      
      currImage = (unsigned char *) camera->getCaptureBuffer();
//       currImage = (unsigned char *) camera.getCaptureBuffer();

//       currImage = (unsigned char *) frame.getImagePtr();
      
      imageYUV = filterYUVQuickCamToYUV.process(currImage, 640* 480*2);
      
      imageRGB = filterYUVToRGB.process(imageYUV, 640* 480*2);

// 	  std::cout << "display" << std::endl;
          //xvDisplay->displayFrameYUV((char*) frame.getImagePtr());
	  xvDisplay->displayFrameRGB((char *) imageRGB);
	  //xvDisplay->displayFrameYUV((char *) currImage);
	  
// 	  camera->captureEnd();
	




    }

    return 0;
}

