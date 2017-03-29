/*
 * $Id: Filter.h 1531 2006-08-01 21:36:57Z phbaer $
 *
 *
 * Copyright 2005,2006 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */
#ifndef SpicaHelper_h
#define SpicaHelper_h 1

#include "ros/ros.h"
#include <msl_sensor_msgs/BallInfo.h>
#include <msl_sensor_msgs/LocalizationType.h>
#include <msl_sensor_msgs/ObstacleInfo.h>
#include <msl_sensor_msgs/VisionControl.h>
#include <msl_sensor_msgs/VisionDebug.h>
#include <msl_sensor_msgs/VisionImage.h>

#include <vector>

using namespace std;

class SpicaHelper
{
  public:
    ros::Subscriber sub;
    ros::Subscriber viSub;
    ros::Publisher VCPub;
    msl_sensor_msgs::VisionDebug *vdd;
    msl_sensor_msgs::BallInfo bi;

    void handleLinepointData(const msl_sensor_msgs::VisionDebug::ConstPtr &msg);
    void handleVisionImage(const msl_sensor_msgs::VisionImage::ConstPtr &msg);
    void sendVisionControl(char key, char debugMode);

    void initialize(const char *nodename, bool imagedata);

    bool lpdirty;
    bool wmdirty;
    bool vidirty;
    int receiverID;
    ros::NodeHandle *nh;
    vector<msl_msgs::Point2dInfo> linePoints;
    vector<msl_sensor_msgs::ObstacleInfo> obstacles;
    vector<double> distanceScan;

    vector<unsigned char> imageData;
    int width, height;
    vector<int> params;

    msl_msgs::PositionInfo pos;
    ros::AsyncSpinner *spinner;
};

#endif
