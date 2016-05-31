#include "ros/ros.h"
#include <OpenNI.h>
#include "OniSampleUtilities.h"

#define MIN_NUM_CHUNKS(data_size, chunk_size)   ((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)  (MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))


using namespace std;
using namespace openni;

int main(int argc, char **argv) {
	ros::init(argc, argv, "astra_pro_driver");

	ros::NodeHandle n;

	ros::Rate loop_rate(10);


	Status rc = OpenNI::initialize();
	Device device;
	VideoStream depth;
	const char* deviceURI = openni::ANY_DEVICE;

    rc = openni::OpenNI::initialize();

    printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

    rc = device.open(deviceURI);
    if (rc != openni::STATUS_OK)
    {
            printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
            openni::OpenNI::shutdown();
            return 1;
    }

    rc = depth.create(device, openni::SENSOR_DEPTH);
    if (rc == openni::STATUS_OK)
    {
            rc = depth.start();
            if (rc != openni::STATUS_OK)
            {
                    printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
                    depth.destroy();
            }
    }
    else
    {
            printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
    }


    openni::VideoFrameRef depthFrame;
    openni::VideoMode depthVideoMode;

    int width;
    int height;
    const long MAX_DEPTH = 10000;
    const long TEXTURE_SIZE = 512;
    float pDepthHist[MAX_DEPTH];
    openni::RGB888Pixel* pTexMap;
    unsigned int nTexMapX;
    unsigned int nTexMapY;

    nTexMapX = MIN_CHUNKS_SIZE(width, TEXTURE_SIZE);
    nTexMapY = MIN_CHUNKS_SIZE(height, TEXTURE_SIZE);
    pTexMap = new openni::RGB888Pixel[nTexMapX * nTexMapY];

    if (depth.isValid())
   {
    	cout << "depth stream is valide!" << endl;

		   depthVideoMode = depth.getVideoMode();
		   width = depthVideoMode.getResolutionX();
		   height = depthVideoMode.getResolutionY();

	    	cout << "size: " << width  << " x " << height << endl;
   } else {
	   cout << "error depth stream is not valid!" << endl;
   }

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();

		depth.readFrame(&depthFrame);

        if (depthFrame.isValid())
        {
                calculateHistogram(pDepthHist, MAX_DEPTH, depthFrame);
        } else {
        	cout << "error! depthFrame is not valid!!!" << endl;
        }

        memset(pTexMap, 0, nTexMapX*nTexMapY*sizeof(openni::RGB888Pixel));


        const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame.getData();
        openni::RGB888Pixel* pTexRow = pTexMap + depthFrame.getCropOriginY() * nTexMapX;
        int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

        openni::CoordinateConverter coorConverter;


        for (int y = 0; y < depthFrame.getHeight(); ++y)
        {
                const openni::DepthPixel* pDepth = pDepthRow;

                openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();

                for (int x = 0; x < depthFrame.getWidth(); ++x, ++pDepth, ++pTex)
                {
                	float worldX, worldY, worldZ = 0.0;
                        if (*pDepth != 0)
                        {

                                int nHistValue = pDepthHist[*pDepth];

                                pTex->r = nHistValue;
                                pTex->g = nHistValue;
                                pTex->b = 0;
                                coorConverter.convertDepthToWorld(depth,x,y,nHistValue,&worldX,&worldY,&worldZ);

//                                cout <<  " x" << worldX << " y" << worldY << " z" << worldZ;
                        }

                }

                pDepthRow += rowSize;
                pTexRow += nTexMapX;
        }
	}

	return 0;
}

