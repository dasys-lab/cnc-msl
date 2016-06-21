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
	};

} /* namespace msl */

#endif /* SRC_FRAMELISTENER_H_ */
