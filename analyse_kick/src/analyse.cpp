/*
 * analyse.cpp
 *
 *  Created on: 11 Mar 2016
 *      Author: emmeda
 */

#include "analyse.h"
#include <iostream>
#include <math.h>
#include <msl_msgs/JoystickCommand.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/LaserScan.h>
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

void onJoystickMsgs(msl_msgs::JoystickCommand msg)
{
    cout << "Received Joystick-Msg" << endl;
    saveReferenceScan = true;
    searchBall = false;
}

void onLaserMsgs(sensor_msgs::LaserScanPtr msg)
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
        for (int i = 180; i < msg->ranges.size() - 180; i++)
        {
            if (referenceScan->ranges[i] - msg->ranges[i] > 0.40 && msg->ranges[i] > 0.30)
            {
                //				cout << "Sector " << i << ": " << referenceScan->ranges[i] << ", " << msg->ranges[i] << (iMatched ? " continues" :
                //"")<< endl;
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
        double angle = (shortestIdx - 180) * 0.25 * M_PI / 180;
        double sign = 1;
        if (angle > M_PI / 2)
        {
            angle = M_PI - angle;
            sign = -1;
        }
        double x = 4.14 - (cos(angle) * (shortestDist + 0.12) * sign);
        double y = (sin(angle) * (shortestDist + 0.12)) + (0.12 - 0.065);

        cout << x << "\t" << y << endl;
    }
}

double getKickPowerExperimental(double dist, double height, double heightTolerance = 30.0)
{
    // scale to meter :)
    dist = dist / 1000.0;
    height = height / 1000.0;
    heightTolerance = heightTolerance / 1000.0;

    double g = 9.81;
    double vSample = 8;
    double vOptimal = 0;
    double heightErr = 100000.0;

    for (int i = 0; i < 100; i++)
    {
        double initialShootAngle = 2.676119513 * vSample + 12.70950743;
        initialShootAngle *= M_PI / 180;
        double y = dist * tan(initialShootAngle) - (g * dist * dist) / (2 * vSample * vSample * cos(initialShootAngle) * cos(initialShootAngle));
        double curHeightErr = abs(height - y);
        if (curHeightErr < heightErr)
        {
            heightErr = curHeightErr;
            vOptimal = vSample;
        }
        else
        {
            break;
        }

        vSample += 0.035; // 100 steps until 11,5m/s
    }

    if (heightErr > heightTolerance)
        return -1;

    // function to map v0 to kickPower
    double f2 = 350.0;
    double f3 = 11.5;
    double f4 = 850.0;

    double kickPower = (-log(1 - (vOptimal / f3)) * f2) + f4;

    return kickPower;
}

int main(int argc, char **argv)
{
    cout << "Evaluation of getKickPowerExperimental" << endl;

    double dist = 1000;
    double height = 200;
    double power = getKickPowerExperimental(dist, height);

    cout << "Dist: " << dist << ", Height: " << height << ", Power: " << power << endl;

    //	for (int i = 1; i < 100; ++i)
    //	{
    //		double dist = (i * 0.2 + 2)*1000;
    //		double height = 800;
    //		double power = getKickPowerExperimental(dist, height);
    //
    //		cout << "Dist: " << dist << ", Height: " << height << ", Power: " << power << endl;
    //	}

    //	cout << "Initialising ROS" << endl;
    //
    //	ros::init(argc, argv, "AnalyseKick");
    //	ros::NodeHandle* nh = new ros::NodeHandle();
    //	ros::AsyncSpinner* spinner = new ros::AsyncSpinner(3);
    //
    //	ros::Subscriber joySub = nh->subscribe("/Joystick", 10, &onJoystickMsgs);
    //	ros::Subscriber laserSub = nh->subscribe("/scan_hokuyo", 10, &onLaserMsgs);
    //
    //	spinner->start();
    //
    //	while (ros::ok())
    //	{
    //		sleep(1);
    //	}
    //
    //	joySub.shutdown();
    //	laserSub.shutdown();
    //	delete nh;
    //	delete spinner;
    //
    return 0;
}
