/*
 * LightBarrier.h
 *
 *  Created on: Apr 18, 2016
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_LIGHTBARRIER_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_LIGHTBARRIER_H_

#include "SystemConfig.h"

namespace msl
{

class MSLWorldModel;
class LightBarrier
{
  public:
    LightBarrier(MSLWorldModel *wm);
    virtual ~LightBarrier();
    bool getLightBarrier(int index = 0);
    bool mayUseLightBarrier();

  private:
    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;
    bool useLightBarrier;
};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_LIGHTBARRIER_H_ */
