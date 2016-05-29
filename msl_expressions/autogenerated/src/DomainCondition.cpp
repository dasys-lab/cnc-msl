#include "DomainCondition.h"
#include <SystemConfig.h>
#include <MSLWorldModel.h>

namespace alica
{
	DomainCondition::DomainCondition() :
			BasicCondition()
	{
		this->wm = msl::MSLWorldModel::get();
		this->sc = supplementary::SystemConfig::getInstance();
		this->timeUntilEmergencyExecute = (*this->sc)["StandardSituation"]->get<double>("Times","timeUntilEmergencyExecute", NULL);
	}

	DomainCondition::~DomainCondition()
	{
	}
} /* namespace alica */
