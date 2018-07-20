/*
 * $Id: BallZTracker.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "ballTracking/BallZTracker.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "ballTracking/TimeHelper.h"

BallZTracker *BallZTracker::instance_ = NULL;

BallZTracker *BallZTracker::getInstance()
{
    if (instance_ == NULL)
        instance_ = new BallZTracker();
    return instance_;
}

BallZTracker::BallZTracker()
{
    init();
}

BallZTracker::~BallZTracker()
{
    cleanup();
}

void BallZTracker::init()
{
}

void BallZTracker::cleanup()
{
}

ZEstimate BallZTracker::trackObject(ObservedPoint *points, int length, int startIndex, int lastIndex)
{
    ZEstimate ze;
    double *zvalues = (double *)malloc(length * sizeof(double));

    for (int i = 0; i < length; i++)
    {
        zvalues[i] = points[i].z;
    }

    double sumTimesSquare = 0.0;
    double sumTimesThree = 0.0;
    double sumTimes = 0.0;
    double sumZ = 0.0;
    double sumTimesZ = 0.0;
    double lambda = 0.0;
    int validCounter = 0;

    unsigned long long timeOmniCam = TimeHelper::getInstance()->getVisionTimeOmniCam();

    if (points[lastIndex].valid)
    {

        sumZ = zvalues[lastIndex];
        unsigned long long timediff = TimeHelper::getInstance()->getTimeDiffToOmniCam(points[lastIndex].timestamp);

        double a = -1.0;
        if (timeOmniCam < points[lastIndex].timestamp)
            a = 1.0;

        sumTimes = a * timediff / 1.0E07;
        sumTimesSquare = (a * timediff / 1.0E07) * (a * timediff / 1.0E07);
        sumTimesThree = (a * timediff / 1.0E07) * (a * timediff / 1.0E07) * (a * timediff / 1.0E07);
        sumTimesZ = zvalues[lastIndex] * (a * timediff / 1.0E07);

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

        sumZ += zvalues[currIndex];
        sumTimes += (a * timediff / 1.0E07);
        sumTimesSquare += (a * timediff / 1.0E07) * (a * timediff / 1.0E07);
        sumTimesThree += (a * timediff / 1.0E07) * (a * timediff / 1.0E07) * (a * timediff / 1.0E07);
        sumTimesZ += zvalues[currIndex] * (a * timediff / 1.0E07);

        currIndex++;
        if (currIndex >= length)
            currIndex -= length;
    }

    ze.z = 120.0;
    ze.vz = 0.0;

    if (validCounter >= 1)
    {

        ze.z = zvalues[lastIndex];
        ze.vz = 0.0;
    }

    if (validCounter >= 4)
    {

        double a = 9810;

        double lambda = 12.0;

        ze.z =
            (sumTimesSquare + lambda) * sumZ - sumTimesZ * sumTimes - 0.5 * a * sumTimesThree * sumTimes + 0.5 * a * sumTimesSquare * (sumTimesSquare + lambda);
        ze.z = ze.z / (validCounter * (sumTimesSquare + lambda) - sumTimes * sumTimes);

        if (ze.z < 120.0)
            ze.z = 120.0;

        lambda = -0.01 * ze.z;

        ze.vz = validCounter * sumTimesZ - sumTimes * sumZ - 0.5 * a * sumTimesSquare * sumTimes + 0.5 * validCounter * a * sumTimesThree;
        ze.vz = ze.vz / (validCounter * (sumTimesSquare + lambda) - sumTimes * sumTimes);
    }

    free(zvalues);

    return ze;
}
