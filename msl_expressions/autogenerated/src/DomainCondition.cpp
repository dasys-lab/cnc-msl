#include "DomainCondition.h"

namespace alica
{
DomainCondition::DomainCondition() :
    BasicCondition()
{
	this->wm = msl::MSLWorldModel::get();
}

DomainCondition::~DomainCondition()
{
}
} /* namespace alica */
