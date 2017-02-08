/*
 * $Id: TimeHelper.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "ballTracking/TimeHelper.h"
#include <iostream>

TimeHelper *TimeHelper::instance = NULL;

TimeHelper::TimeHelper()
    : sc()
{

    this->sc = SystemConfig::getInstance();
}

TimeHelper::~TimeHelper()
{
}

TimeHelper *TimeHelper::getInstance()
{

    if (instance == NULL)
    {
        instance = new TimeHelper();
    }

    return instance;
}

void TimeHelper::setVisionTimeOmniCam(unsigned long long time_)
{
    visionTimeOmniCam = time_;
}

void TimeHelper::setVisionTimeDirected(unsigned long long time_)
{
    visionTimeDirected = time_;
}

unsigned long long TimeHelper::getVisionTimeOmniCam()
{
    return visionTimeOmniCam;
}

unsigned long long TimeHelper::getVisionTimeDirected()
{
    return visionTimeDirected;
}

unsigned long long TimeHelper::getTimeDiffToOmniCam(unsigned long long time_)
{

    if (time_ > visionTimeOmniCam)
        return (time_ - visionTimeOmniCam);
    else
        return (visionTimeOmniCam - time_);
}

unsigned long long TimeHelper::getTimeDiff(unsigned long long time1, unsigned long long time2)
{

    if (time2 > time1)
        return (time2 - time1);
    else
        return (time1 - time2);
}
