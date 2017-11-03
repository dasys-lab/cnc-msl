#pragma once

#include <BehaviourCreator.h>
#include <ConditionCreator.h>
#include <ConstraintCreator.h>
#include <MSLWorldModel.h>
#include <UtilityFunctionCreator.h>
#include <engine/AlicaEngine.h>

#include <iostream>

namespace supplementary {
	class AgentIDManager;
}

namespace msl
{

class Base
{
  public:
    Base(std::string roleSetName, std::string masterPlanName, std::string roleSetDir, bool sim);
    virtual ~Base();

    void start();

    alica::AlicaEngine *ae;
    alica::BehaviourCreator *bc;
    alica::ConditionCreator *cc;
    alica::UtilityFunctionCreator *uc;
    alica::ConstraintCreator *crc;
    supplementary::AgentIDManager* idManager;
    MSLWorldModel *wm;
};

} /* namespace msl */
