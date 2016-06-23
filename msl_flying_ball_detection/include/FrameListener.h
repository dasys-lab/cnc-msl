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

		void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud, ros::Publisher pub);
		RingBuffer<InformationElement<Eigen::Vector4d> > flyingBallPositions;
		void checkBallTrajectory(unsigned long  time);
		ros::Publisher kickControlPub;

	};

} /* namespace msl */

#endif /* SRC_FRAMELISTENER_H_ */
