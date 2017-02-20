/*
 * Base.h
 *
 *  Created on: 22.10.2014
 *      Author: Andreas Witsch
 */

#ifndef BASE_H_
#define BASE_H_

#include <iostream>

#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "MSLWorldModel.h"
#include "UtilityFunctionCreator.h"
#include "engine/AlicaEngine.h"

using namespace std;

namespace msl
{

class Base
{
  public:
    Base(string roleSetName, string masterPlanName, string roleSetDir, bool sim);
    virtual ~Base();

    void start();

    alica::AlicaEngine *ae;
    alica::BehaviourCreator *bc;
    alica::ConditionCreator *cc;
    alica::UtilityFunctionCreator *uc;
    alica::ConstraintCreator *crc;
    MSLWorldModel *wm;
};

} /* namespace msl */

#endif /* BASE_H_ */
