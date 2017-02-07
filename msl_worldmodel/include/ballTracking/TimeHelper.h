/*
 * $Id: TimeHelper.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef TimeHelper_H
#define TimeHelper_H

#include <SystemConfig.h>

using namespace supplementary;

class TimeHelper
{

  public:
    static TimeHelper *getInstance();

    unsigned long long getVisionTimeOmniCam();
    unsigned long long getVisionTimeDirected();

    void setVisionTimeOmniCam(unsigned long long time_);
    void setVisionTimeDirected(unsigned long long time_);

    unsigned long long getTimeDiffToOmniCam(unsigned long long time_);

    static unsigned long long getTimeDiff(unsigned long long time1, unsigned long long time2);

  private:
    SystemConfig *sc;

    static TimeHelper *instance;
    TimeHelper();
    ~TimeHelper();

    unsigned long long visionTimeOmniCam;
    unsigned long long visionTimeDirected;
};

#endif
