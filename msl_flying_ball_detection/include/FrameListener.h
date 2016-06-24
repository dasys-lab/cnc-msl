/*
 * FrameListener.h
 *
 *  Created on: Jun 19, 2016
 *      Author: Stephan Opfer
 */

#ifndef SRC_FRAMELISTENER_H_
#define SRC_FRAMELISTENER_H_

#include <OpenNI.h>
#include "ros/ros.h"
#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include <InformationElement.h>
#include <RingBuffer.h>

namespace msl_msgs{
	ROS_DECLARE_MESSAGE(RefBoxCommand)
}

namespace msl
{

	class FrameListener : public openni::VideoStream::NewFrameListener
	{
	public:
		FrameListener();
		virtual ~FrameListener();
		void onNewFrame(openni::VideoStream&);

	private:
		ros::NodeHandle rosNode;
		ros::Publisher pub;
		ros::Publisher pubBall;
		ros::Publisher kickControlPub;
		ros::Subscriber refBoxCommandSub;
		RingBuffer<InformationElement<Eigen::Vector4d> > flyingBallPositions;
		bool mayKick;

		void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud, ros::Publisher pub);
		void checkBallTrajectory(unsigned long  time);
		bool fillCloudFromDepth(openni::VideoStream& vidStream, pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud);
		void createVoxelCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leafSize, pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud);
		void onRefBoxCommand(msl_msgs::RefBoxCommandPtr msg);
		void kick();

	};

} /* namespace msl */

#endif /* SRC_FRAMELISTENER_H_ */
