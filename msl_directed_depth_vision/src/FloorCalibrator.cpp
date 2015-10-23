/*
 * FloorCalibrator.cpp
 *
 *  Created on: 23.10.2015
 *      Author: tobi
 */

#include "ros/ros.h"


namespace msl_vision
{

	int main(int argc, char **argv) {
		ros::init(argc, argv, "FloorCalibrator");
		ros::NodeHandle n;

		ros::Rate loop_rate(10);
		while (ros::ok()) {

			ros::spinOnce();
			loop_rate.sleep();

		}
		return 0;

	}


} /* namespace msl_vision */
