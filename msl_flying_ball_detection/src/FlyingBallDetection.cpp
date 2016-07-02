#include "ros/ros.h"
#include <OpenNI.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include "FrameListener.h"

using namespace std;

int main(int argc, char **argv)
{
	openni::Device device;
	openni::VideoStream depth;
	const char* deviceURI = openni::ANY_DEVICE;

	openni::OpenNI::initialize();

	cout << "FlyingBallDetection: After initialization:" << endl << openni::OpenNI::getExtendedError() << endl;

	if (openni::STATUS_OK != device.open(deviceURI))
	{
		cout << "FlyingBallDetection: Device open failed:"<< endl << openni::OpenNI::getExtendedError() << endl;
		openni::OpenNI::shutdown();
		return 1;
	}

	if (openni::STATUS_OK != depth.create(device, openni::SENSOR_DEPTH))
	{
		cout << "FlyingBallDetection: Couldn't find depth stream:" << endl << openni::OpenNI::getExtendedError() << endl;
		openni::OpenNI::shutdown();
		return 1;
	}

	for (int i = 0; i < device.getSensorInfo(openni::SENSOR_DEPTH)->getSupportedVideoModes().getSize(); i++)
	{
		openni::VideoMode mode = device.getSensorInfo(openni::SENSOR_DEPTH)->getSupportedVideoModes()[i];
		cout << i << ": " << mode.getResolutionX() << " x " << mode.getResolutionY() << " " << mode.getFps() << " "
				<< mode.getPixelFormat() << endl;
	}

	if (openni::STATUS_OK != depth.setVideoMode(device.getSensorInfo(openni::SENSOR_DEPTH)->getSupportedVideoModes()[2]))// 4 -> 1_MM 5 -> 100_UM
	{
		cout << "FlyingBallDetection: Couldn't set video mode:" << endl << openni::OpenNI::getExtendedError() << endl;
		depth.destroy();
		openni::OpenNI::shutdown();
		return 1;
	}

	if (openni::STATUS_OK != depth.setMirroringEnabled(false))
	{
		cout << "FlyingBallDetection: Couldn't set mirroring to false:" << endl << openni::OpenNI::getExtendedError() << endl;
		depth.destroy();
		openni::OpenNI::shutdown();
		return 1;
	}

	if (openni::STATUS_OK != depth.start())
	{
		cout << "FlyingBallDetection: Couldn't start depth stream:" << endl << openni::OpenNI::getExtendedError() << endl;
		depth.destroy();
		openni::OpenNI::shutdown();
		return 1;
	}

	ros::init(argc, argv, "flying_ball_detection");
	ros::AsyncSpinner spinner(4);
	spinner.start();
	msl::FrameListener* fl = new msl::FrameListener();
	depth.addNewFrameListener(fl);

	while (ros::ok())
	{
		boost::this_thread::sleep (boost::posix_time::microseconds (500000));
	}

	spinner.stop();

	delete fl;
	return 0;
}

