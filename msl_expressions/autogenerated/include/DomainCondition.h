#pragma once
#include "engine/BasicCondition.h"

namespace msl{
	class MSLWorldModel;
	class MSLRobot;
	class Rules;
}

namespace supplementary {
	class SystemConfig;
}

namespace alica
{
	class DomainCondition : public BasicCondition
	{
	public:
		DomainCondition();
		virtual ~DomainCondition();

		msl::MSLWorldModel* wm;
		msl::MSLRobot* robot;
		supplementary::SystemConfig* sc;
		msl::Rules* rules;

		/* The time that can pass since the last start command
		 * until we need to skip proper positioning and execute
		 * the standard right away. [ns]
		 */
		uint64_t timeUntilEmergencyExecute;
	};
} /* namespace alica */


