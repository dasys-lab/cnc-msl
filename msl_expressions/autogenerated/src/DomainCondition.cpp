#include "DomainCondition.h"
#include <SystemConfig.h>
#include <MSLWorldModel.h>
#include <msl_robot/MSLRobot.h>
#include <Rules.h>

namespace alica
{
	DomainCondition::DomainCondition() :
			BasicCondition()
	{
		this->wm = msl::MSLWorldModel::get();
		this->robot = msl::MSLRobot::get();
		this->rules = msl::Rules::getInstance();
		this->sc = supplementary::SystemConfig::getInstance();
		this->timeUntilEmergencyExecute = this->rules->getStandbyTime() *0.8;
	}

	DomainCondition::~DomainCondition()
	{
	}
} /* namespace alica */
