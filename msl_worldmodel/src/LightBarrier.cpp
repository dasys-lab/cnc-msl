/*
 * LightBarrier.cpp
 *
 *  Created on: Apr 18, 2016
 *      Author: Stefan Jakob
 */

#include "MSLWorldModel.h"
#include "RawSensorData.h"
#include <LightBarrier.h>

using nonstd::optional;
using nonstd::nullopt;

namespace msl
{

LightBarrier::LightBarrier(MSLWorldModel *wm)
{
    this->wm = wm;
    this->sc = supplementary::SystemConfig::getInstance();
    this->useLightBarrier = (*this->sc)["LightBarrier"]->get<bool>("LightBarrier", "UseLightBarrier", NULL);
}

LightBarrier::~LightBarrier()
{
}

optional<bool> LightBarrier::getLightBarrier()
{
    auto lbInfo = this->wm->rawSensorData->getLightBarrierBuffer().getLastValid();

    if (lbInfo)
    {
        return lbInfo->getInformation();
    }

    return nullopt;
}

bool LightBarrier::mayUseLightBarrier()
{
    return this->useLightBarrier;
}

} /* namespace msl */
