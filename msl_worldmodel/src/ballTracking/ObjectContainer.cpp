/*
 * $Id: ObjectContainer.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "ballTracking/ObjectContainer.h"
#include "ballTracking/TimeHelper.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

ObjectContainer::ObjectContainer(int size_)
{

    size = size_;

    points = (ObservedPoint *)malloc(size * sizeof(ObservedPoint));
    memset(points, 0, size * sizeof(ObservedPoint));

    startIndex = 0;
    lastIndex = 0;
    lastValidIndex = 0;
    points[0].valid = false;

    validCounter = 0;

    init();
}

ObjectContainer::~ObjectContainer()
{
    cleanup();
}

void ObjectContainer::init()
{
}

void ObjectContainer::cleanup()
{

    free(points);
}

void ObjectContainer::integratePoint(ObservedPoint p)
{
    if (lastIndex != startIndex || (lastIndex == startIndex && validCounter == 1))
    {
        lastIndex++;
        if (lastIndex >= size)
            lastIndex -= size;
    }
    points[lastIndex] = p;

    lastValidIndex = -1;
    validCounter = 0;
    if (points[lastIndex].valid)
    {
        validCounter++;
        lastValidIndex = lastIndex;
    }

    int currIndex = startIndex;

    while (currIndex != lastIndex)
    {

        if (points[currIndex].valid)
        {
            validCounter++;
            if (lastValidIndex != lastIndex)
                lastValidIndex = currIndex;
        }

        currIndex++;
        if (currIndex >= size)
            currIndex -= size;
    }
}

void ObjectContainer::invalidate(int ms)
{
    int currIndex = startIndex;

    unsigned long long timeDiff = 0;

    while (currIndex != lastIndex)
    {
        timeDiff = TimeHelper::getInstance()->getTimeDiffToOmniCam(points[currIndex].timestamp);
        if (timeDiff > ms * 10000)
        {
            points[currIndex].valid = false;
            startIndex = currIndex + 1;
            if (startIndex >= size)
                startIndex -= size;
        }

        currIndex++;
        if (currIndex >= size)
            currIndex -= size;
    }

    timeDiff = TimeHelper::getInstance()->getTimeDiffToOmniCam(points[lastIndex].timestamp);

    if (timeDiff > ms * 10000)
    {
        points[lastIndex].valid = false;
        startIndex = lastIndex;
    }

    validCounter = 0;
    lastValidIndex = -1;
    if (points[lastIndex].valid)
    {
        validCounter++;
        lastValidIndex = lastIndex;
    }

    currIndex = startIndex;

    while (currIndex != lastIndex)
    {

        if (points[currIndex].valid)
        {
            validCounter++;
            if (lastValidIndex != lastIndex)
                lastValidIndex = currIndex;
        }

        currIndex++;
        if (currIndex >= size)
            currIndex -= size;
    }
}

void ObjectContainer::reset()
{

    startIndex = 0;
    lastIndex = 0;
    lastValidIndex = 0;
    validCounter = 0;
    points[0].valid = false;
}

int ObjectContainer::getSize()
{

    return size;
}

int ObjectContainer::getStartIndex()
{

    return startIndex;
}

int ObjectContainer::getLastIndex()
{

    return lastIndex;
}

int ObjectContainer::getNumberValidPoints()
{

    return validCounter;
}

ObservedPoint *ObjectContainer::getPoints()
{

    return points;
}
