#include "ros/ros.h"
#include <OpenNI.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include "FrameListener.h"
#include <chrono>

#include <thread>

#define MIN_NUM_CHUNKS(data_size, chunk_size)   ((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)  (MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

using namespace std;
using namespace openni;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "astra_pro_driver");

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
	msl::FrameListener* fl = new msl::FrameListener();
	depth.addNewFrameListener(fl);

	if (rc == openni::STATUS_OK)
	{

		for (int i = 0; i < device.getSensorInfo(SENSOR_DEPTH)->getSupportedVideoModes().getSize(); i++)
		{
			VideoMode mode = device.getSensorInfo(SENSOR_DEPTH)->getSupportedVideoModes()[i];
			cout << i << ": " << mode.getResolutionX() << " x " << mode.getResolutionY() << " " << mode.getFps() << " "
					<< mode.getPixelFormat() << endl;

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
		cout << "depth stream is valid!" << endl;
		depthVideoMode = depth.getVideoMode();
		width = depthVideoMode.getResolutionX();
		height = depthVideoMode.getResolutionY();

		cout << "size: " << width << " x " << height << endl;
	}
	else
	{
		cout << "error depth stream is not valid!" << endl;
	}

	while (ros::ok())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	delete fl;
	return 0;
}

