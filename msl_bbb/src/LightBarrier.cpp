/*
 * lightbarrier.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: Lukas Will
 */

#include "LightBarrier.h"

#include <BlackADC.h>
#include <SystemConfig.h>
#include <std_msgs/Bool.h>

namespace msl_bbb
{

LightBarrier::LightBarrier(BlackLib::adcName adc_P, Communication *comm) : Worker("LightBarrier")
{
    this->comm = comm;
    adc = new BlackLib::BlackADC(adc_P);

    auto sc = supplementary::SystemConfig::getInstance();
    this->threshold = (*sc)["bbb"]->get<int>("BBB.lightbarrierThreshold", NULL);
}

LightBarrier::~LightBarrier()
{
    delete adc;
}

bool LightBarrier::checkLightBarrier()
{
    if (adc->getNumericValue() > threshold)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool LightBarrier::setTreshold(int th)
{
    if ((th > 0) && (th < 1024))
    {
        threshold = th;
        return true;
    }
    else
    {
        return false;
    }
}

void LightBarrier::run()
{
    std_msgs::Bool msg;
    msg.data = this->checkLightBarrier();
    this->comm->onRosBool2802967882(msg);
}
}
