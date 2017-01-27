/*
 * GoalDetection.cpp
 *
 *  Created on: Jan 27, 2017
 *      Author:  Lisa Martmann
 */

#include "ros/ros.h"
#include "LaserScanListener.h"
#include <thread>
#include <chrono>


using namespace std;


int main(int argc, char *argv[])
{
	std::string node_name = "goal_detection";
	ros::init(argc, argv, node_name);
	ros::AsyncSpinner spinner(4);
	spinner.start();
	msl::LaserScanListener* lsl = new msl::LaserScanListener();

	while (ros::ok())
	{
		std::chrono::milliseconds dura(500);
		std::this_thread::sleep_for(dura);
	}

	spinner.stop();

	delete lsl;

	return 0;
}
