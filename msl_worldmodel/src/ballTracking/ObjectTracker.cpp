/*
 * $Id: ObjectTracker.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "ballTracking/ObjectTracker.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "ballTracking/TimeHelper.h"

ObjectTracker *ObjectTracker::instance_ = NULL;

ObjectTracker *ObjectTracker::getInstance()
{

    if (instance_ == NULL)
        instance_ = new ObjectTracker();
    return instance_;
}

ObjectTracker::ObjectTracker()
{
    init();
}

ObjectTracker::~ObjectTracker()
{
    cleanup();
}

void ObjectTracker::init()
{
}

void ObjectTracker::cleanup()
{
}

MovingObject ObjectTracker::trackObject(ObservedPoint *points, int length, int startIndex, int lastIndex, double rotVelocity)
{
    MovingObject mv;
    Point *pointsTmp = (Point *)malloc(length * sizeof(Point));

    for (int i = 0; i < length; i++)
    {
        pointsTmp[i].x = points[i].x;
        pointsTmp[i].y = points[i].y;
    }

    double sumTimesSquare = 0.0;
    double sumTimes = 0.0;
    Point sumPoints;
    sumPoints.x = 0.0;
    sumPoints.y = 0.0;
    Point sumTimePoints;
    sumTimePoints.x = 0.0;
    sumTimePoints.y = 0.0;
    double lambda = 0.0;
    int validCounter = 0;
    unsigned long long timeOmniCam = TimeHelper::getInstance()->getVisionTimeOmniCam();

    if (points[lastIndex].valid)
    {
        sumPoints.x = pointsTmp[lastIndex].x;
        sumPoints.y = pointsTmp[lastIndex].y;
        unsigned long long timediff = TimeHelper::getInstance()->getTimeDiffToOmniCam(points[lastIndex].timestamp);

        double a = -1.0;
        if (timeOmniCam < points[lastIndex].timestamp)
            a = 1.0;

        sumTimes = a * timediff / 1.0E07;
        sumTimesSquare = (a * timediff / 1.0E07) * (a * timediff / 1.0E07);
        sumTimePoints.x += pointsTmp[lastIndex].x * (a * timediff / 1.0E07);
        sumTimePoints.y += pointsTmp[lastIndex].y * (a * timediff / 1.0E07);

        validCounter++;
    }

    int currIndex = startIndex;

    while (currIndex != lastIndex)
    {
        if (!points[currIndex].valid)
        {
            currIndex++;
            if (currIndex >= length)
                currIndex -= length;
            continue;
        }
        validCounter++;

        unsigned long long timediff = TimeHelper::getInstance()->getTimeDiffToOmniCam(points[currIndex].timestamp);

        double a = -1.0;
        if (timeOmniCam < points[currIndex].timestamp)
            a = 1.0;

        sumTimesSquare += (a * timediff / 1.0E07) * (a * timediff / 1.0E07);
        sumTimes += (a * timediff / 1.0E07);
        sumPoints.x += pointsTmp[currIndex].x;
        sumPoints.y += pointsTmp[currIndex].y;

        sumTimePoints.x += pointsTmp[currIndex].x * (a * timediff / 1.0E07);
        sumTimePoints.y += pointsTmp[currIndex].y * (a * timediff / 1.0E07);

        currIndex++;
        if (currIndex >= length)
            currIndex -= length;
    }
    rotVelocity = fabs(rotVelocity);
    if (rotVelocity > 0.5)
        rotVelocity = 0.5;

    double lambdaAdd = 0.0; // rotVelocity*2.0*0.5;
    double powerAdd = 0.0;  // rotVelocity*2.0*0.45;

    if (validCounter <= 3)
        lambda = 0.5 + lambdaAdd;
    else
        lambda = (0.5 + lambdaAdd) * pow((0.75 + powerAdd), validCounter - 3);

    Point p0;
    p0.x = 100000.0;
    p0.y = 100000.0;

    Velocity v;
    v.vx = 0.0;
    v.vy = 0.0;

    if (validCounter >= 1)
    {
        p0.x = ((lambda + sumTimesSquare) * sumPoints.x - sumTimes * sumTimePoints.x) / (validCounter * (lambda + sumTimesSquare) - sumTimes * sumTimes);
        p0.y = ((lambda + sumTimesSquare) * sumPoints.y - sumTimes * sumTimePoints.y) / (validCounter * (lambda + sumTimesSquare) - sumTimes * sumTimes);
    }

    if (validCounter >= 2)
    {
        v.vx = (validCounter * sumTimePoints.x - sumTimes * sumPoints.x) / (validCounter * (lambda + sumTimesSquare) - sumTimes * sumTimes);
        v.vy = (validCounter * sumTimePoints.y - sumTimes * sumPoints.y) / (validCounter * (lambda + sumTimesSquare) - sumTimes * sumTimes);
    }

    mv.point = p0;
    mv.velocity = v;
    free(pointsTmp);
    return mv;
}
