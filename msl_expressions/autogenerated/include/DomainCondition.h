#ifndef DomainBehaviour_H_
#define DomainBehaviour_H_

#include "engine/BasicCondition.h"
/*PROTECTED REGION ID(domainHeaderAdditional) ENABLED START*/
namespace msl{
	class MSLWorldModel;
	class MSLRobot;
	class Rules;
}

namespace supplementary {
	class SystemConfig;
}
/*PROTECTED REGION END*/

namespace alica {
class DomainCondition : public BasicCondition {
public:
  DomainCondition();
  virtual ~DomainCondition();

  /*PROTECTED REGION ID(domainHeader) ENABLED START*/
  		msl::MSLWorldModel* wm;
		msl::MSLRobot* robot;
		supplementary::SystemConfig* sc;
		msl::Rules* rules;

		/* The time that can pass since the last start command
		 * until we need to skip proper positioning and execute
		 * the standard right away. [ns]
		 */
		uint64_t timeUntilEmergencyExecute;
  /*PROTECTED REGION END*/
};
} /* namespace alica */

#endif /* DomainBehaviour_H_ */
