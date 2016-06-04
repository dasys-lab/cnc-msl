/*
 * DirectedDepthVisionNode.cpp
 *
 *  Created on: 28.10.2015
 *      Author: Tobias Schellien
 */

#include "ros/ros.h"
#include <mrpt/math/geometry.h>
#include <OpenNI.h>
#include "OniSampleUtilities.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include <DepthVision.h>

#define MIN_NUM_CHUNKS(data_size, chunk_size)   ((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)  (MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

using namespace std;
using namespace openni;

int main(int argc, char **argv) {
	ros::init(argc, argv, "DirectedDepthVisionNode");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud>("/astra/depthCloud", 10000);

	Status rc = OpenNI::initialize();
	Device device;
	VideoStream depth;
	DepthVision depthVision;

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

        for(int i = 0;  i < device.getSensorInfo(SENSOR_DEPTH)->getSupportedVideoModes().getSize(); i++) {
        	VideoMode mode = device.getSensorInfo(SENSOR_DEPTH)->getSupportedVideoModes()[i];
        	cout << i << ": " << mode.getResolutionX() << " x " << mode.getResolutionY() << " " << mode.getFps() << " " << mode.getPixelFormat() <<  endl;

        }
        rc = depth.setVideoMode(device.getSensorInfo(SENSOR_DEPTH)->getSupportedVideoModes()[4]); // 4 -> 1_MM 5 -> 100_UM
        depth.setMirroringEnabled(false);
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

    double width;
    double height;
    const long MAX_DEPTH = 100000;
    float pDepthHist[MAX_DEPTH];

    double horDegree = 60;
    double verDegree = 49.5;

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

        int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);
        sensor_msgs::PointCloud pcl;
        vector< geometry_msgs::Point32> points;
        vector< sensor_msgs::ChannelFloat32> channel;
        sensor_msgs::ChannelFloat32 depthChannel;
        depthChannel.name = "depth";

        const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame.getData();

        for (int y = 0; y < depthFrame.getHeight(); ++y)
        {
                const openni::DepthPixel* pDepth = pDepthRow;

                for (int x = 0; x < depthFrame.getWidth(); ++x, ++pDepth)
                {
                	float worldX, worldY, worldZ = 0.0;
                        if (*pDepth != 0)
                        {
                                int nHistValue = pDepthHist[*pDepth];
                                double roh = (((double)y/height * verDegree)-verDegree/2) * M_PI / 180;
                                double phi = (((double)x/width * horDegree)-horDegree/2) * M_PI / 180;

                                geometry_msgs::Point32 p;
                                double depthM = (*pDepth) / (double)1000;
                                p.z = (sin(roh) * depthM) * -1;
                                p.y = sin(phi) * depthM;
                                p.x = depthM;

                                points.push_back(p);
                                depthChannel.values.push_back(nHistValue);
                        }
                }

                pDepthRow += rowSize;
        }

        channel.push_back(depthChannel);
        pcl.channels = channel;
        pcl.points = points;
        pcl.header.frame_id = "camera_frame";
        pcl.header.stamp = ros::Time::now();

        pub.publish(pcl);


        depthVision.pointCloudCallback(pcl);


	}

	return 0;
}

