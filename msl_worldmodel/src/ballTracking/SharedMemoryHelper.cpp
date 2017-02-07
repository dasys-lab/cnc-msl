/*
 * $Id: SharedMemoryHelper.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "ballTracking/SharedMemoryHelper.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "ballTracking/TrackingPackets.h"

SharedMemoryHelper *SharedMemoryHelper::instance_ = NULL;

SharedMemoryHelper *SharedMemoryHelper::getInstance()
{

    if (instance_ == NULL)
        instance_ = new SharedMemoryHelper();

    return instance_;
}

SharedMemoryHelper::SharedMemoryHelper()
    : opDirectedShmInfo()
    , opKinectShmInfo()
    , coShmInfo()
{
    bzero(opDirected, sizeof(ObservedPoint) * 10);
    bzero(opKinect, sizeof(ObservedPoint) * 10);
    init();
}

SharedMemoryHelper::~SharedMemoryHelper()
{
    cleanup();
}

void SharedMemoryHelper::init()
{
}

void SharedMemoryHelper::cleanup()
{
}

void SharedMemoryHelper::writeDirectedBallPosition(ObservedPoint *p)
{
    this->opDirectedShmInfo.waitForAndLock();
    memcpy(this->opDirectedShmInfo.get(), p, sizeof(ObservedPoint) * 10);
    this->opDirectedShmInfo.unlock();
}

ObservedPoint *SharedMemoryHelper::readDirectedBallPosition()
{
    this->opDirectedShmInfo.waitForAndLock();
    memcpy(opDirected, this->opDirectedShmInfo.get(), sizeof(ObservedPoint) * 10);
    this->opDirectedShmInfo.unlock();
    return opDirected;
}

void SharedMemoryHelper::writeKinectBallPosition(ObservedPoint *p)
{
    this->opKinectShmInfo.waitForAndLock();
    memcpy(this->opKinectShmInfo.get(), p, sizeof(ObservedPoint) * 10);
    this->opKinectShmInfo.unlock();
}

ObservedPoint *SharedMemoryHelper::readKinectBallPosition()
{
    this->opKinectShmInfo.waitForAndLock();
    memcpy(opKinect, this->opKinectShmInfo.get(), sizeof(ObservedPoint) * 10);
    this->opKinectShmInfo.unlock();
    return opKinect;
}

void SharedMemoryHelper::writeCorrectedOdometry(CorrectedOdometry *co)
{
    this->coShmInfo.waitForAndLock();
    memcpy(this->coShmInfo.get(), co, sizeof(CorrectedOdometry));
    this->coShmInfo.unlock();
}

CorrectedOdometry *SharedMemoryHelper::readCorrectedOdometry()
{
    this->coShmInfo.waitForAndLock();
    memcpy(co, this->coShmInfo.get(), sizeof(CorrectedOdometry));
    this->coShmInfo.unlock();
    return co;
}
