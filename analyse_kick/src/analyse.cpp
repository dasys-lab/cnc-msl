/*
 * analyse.cpp
 *
 *  Created on: 11 Mar 2016
 *      Author: emmeda
 */

#include "analyse.h"
#include <iostream>
#include <ros/ros.h>
#include <msl_msgs/JoystickCommand.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/spinner.h>
#include <math.h>
analyse::analyse()
{
	// TODO Auto-generated constructor stub

}

analyse::~analyse()
{
	// TODO Auto-generated destructor stub
}

using namespace std;
bool searchBall = false;
bool saveReferenceScan = false;

sensor_msgs::LaserScanPtr referenceScan;

void onJoystickMsgs (msl_msgs::JoystickCommand msg)
{
	cout << "Received Joystick-Msg" << endl;
	saveReferenceScan = true;
	searchBall = false;
}

void onLaserMsgs (sensor_msgs::LaserScanPtr msg)
{
	if (saveReferenceScan)
	{
		cout << "Reference Scan saved" << endl;
		referenceScan = msg;
		searchBall = true;
		saveReferenceScan = false;
	}
	else if (searchBall)
	{
//		cout << "---------------- Search Ball --------------" << endl;
		bool iMatched = false;
		int shortestIdx = -1;
		double shortestDist = 10000;
		for (int i = 180; i < msg->ranges.size()-180; i++)
		{
			if (referenceScan->ranges[i] - msg->ranges[i] > 0.40 && msg->ranges[i] > 0.30)
			{
//				cout << "Sector " << i << ": " << referenceScan->ranges[i] << ", " << msg->ranges[i] << (iMatched ? " continues" : "")<< endl;
				iMatched = true;

				if (shortestDist > msg->ranges[i])
				{
					shortestIdx = i;
					shortestDist = msg->ranges[i];
				}
			}
			else
			{
				iMatched = false;
			}
		}

		if (shortestIdx == -1)
			return;

		// idx 180 = 0°
		double angle = (shortestIdx-180)*0.25*M_PI/180;
		double sign = 1;
		if (angle > M_PI/2)
		{
			angle = M_PI - angle;
			sign = -1;
		}
		double x = 4.14 - (cos(angle) * (shortestDist+0.12) * sign);
		double y = (sin(angle) * (shortestDist+0.12)) + (0.12-0.065);

		cout << x << "\t" << y << endl;
	}
}

int main(int argc, char** argv)
{
	cout << "Initialising ROS" << endl;

	ros::init(argc, argv, "AnalyseKick");
	ros::NodeHandle* nh = new ros::NodeHandle();
	ros::AsyncSpinner* spinner = new ros::AsyncSpinner(3);

	ros::Subscriber joySub = nh->subscribe("/Joystick", 10, &onJoystickMsgs);
	ros::Subscriber laserSub = nh->subscribe("/scan_hokuyo", 10, &onLaserMsgs);

	spinner->start();

	while (ros::ok())
	{
		sleep(1);
	}

	joySub.shutdown();
	laserSub.shutdown();
	delete nh;
	delete spinner;

	return 0;
}


