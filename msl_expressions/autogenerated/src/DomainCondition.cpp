#include "DomainCondition.h"
/*PROTECTED REGION ID(domainSourceHeaders) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica {
DomainCondition::DomainCondition() : BasicCondition() {
  /*PROTECTED REGION ID(domainSourceConstructor) ENABLED START*/
  		this->wm = msl::MSLWorldModel::get();
		this->robot = msl::MSLRobot::get();
		this->rules = msl::Rules::getInstance();
		this->sc = supplementary::SystemConfig::getInstance();
		this->timeUntilEmergencyExecute = this->rules->getStandbyTime() *0.8;
  /*PROTECTED REGION END*/
}

DomainCondition::~DomainCondition() {
  /*PROTECTED REGION ID(domainSourceDestructor) ENABLED START*/
  // Add additional options here
  /*PROTECTED REGION END*/
}
} /* namespace alica */
