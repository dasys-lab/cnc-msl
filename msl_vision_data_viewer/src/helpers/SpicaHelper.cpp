/*
 * $Id: Filter.cpp 1804 2007-01-07 21:53:55Z saur $
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
#include "SpicaHelper.h"
#include <msl/robot/IntRobotID.h>
#include <supplementary/IAgentID.h>
#include <iostream>

using namespace std;

void SpicaHelper::handleLinepointData(const msl_sensor_msgs::VisionDebug::ConstPtr &msg)
{

	auto tmpID = factory.create(msg->senderID.id);
    if (tmpID != receiverID)
    {
    	delete tmpID;
        return;
    }
    if (msg->locType.type == msl_sensor_msgs::LocalizationType::ParticleFilter)
        cout << "P";
    else
        cout << "E";
    linePoints = msg->list;
    pos = msg->position;
    distanceScan = msg->distanceScan.sectors;
    obstacles = msg->obstacles;
    bi = msg->ball;
    lpdirty = true;
}

void SpicaHelper::handleVisionImage(const msl_sensor_msgs::VisionImage::ConstPtr &msg)
{
    if (vidirty)
        return;
	auto tmpID = factory.create(msg->senderID.id);
    if (tmpID != receiverID)
    {
    	delete tmpID;
        return;
    }
    imageData = msg->imageData;
    height = msg->height;
    width = msg->width;
    params = msg->params;
    vidirty = true;
}

void SpicaHelper::initialize(const char *nodename, bool imagedata)
{
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, nodename);
    lpdirty = false;
    wmdirty = false;
    vidirty = false;

    nh = new ros::NodeHandle();
    sub = nh->subscribe<msl_sensor_msgs::VisionDebug, SpicaHelper>("CNVision/VisionDebug", 1, &SpicaHelper::handleLinepointData, (this),
                                                                   ros::TransportHints().udp());
    if (imagedata)
        viSub = nh->subscribe<msl_sensor_msgs::VisionImage, SpicaHelper>("CNVision/VisionImage", 1, &SpicaHelper::handleVisionImage, (this),
                                                                         ros::TransportHints().udp());
    VCPub = nh->advertise<msl_sensor_msgs::VisionControl>("CNVision/VisionControl", 1);

    spinner = new ros::AsyncSpinner(1);
    spinner->start();
}

void SpicaHelper::sendVisionControl(char key, char debugMode)
{
    msl_sensor_msgs::VisionControl vc;
    vc.key = key;
    vc.debugMode = debugMode;
    vc.receiverID.id = receiverID->toByteVector();
    VCPub.publish(vc);
}
