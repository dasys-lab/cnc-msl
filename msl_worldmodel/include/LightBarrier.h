/*
 * LightBarrier.h
 *
 *  Created on: Apr 18, 2016
 *      Author: Stefan Jakob
 */

#pragma once

#include "SystemConfig.h"

#include <nonstd/optional.hpp>

namespace msl
{

class MSLWorldModel;
class LightBarrier
{
  public:
    LightBarrier(MSLWorldModel *wm);
    virtual ~LightBarrier();
    nonstd::optional<bool> getLightBarrier();
    bool mayUseLightBarrier();

  private:
    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;
    bool useLightBarrier;
};

} /* namespace msl */
