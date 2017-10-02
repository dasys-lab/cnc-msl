#pragma once

#include "IntRobotID.h"

#include <supplementary/IAgentIDFactory.h>

#include <vector>

namespace supplementary {
	class IAgentID;
}

namespace msl
{
namespace robot
{

class IntRobotIDFactory : public supplementary::IAgentIDFactory
{
  public:
    IntRobotIDFactory();
    virtual ~IntRobotIDFactory();

    const supplementary::IAgentID* create(std::vector<uint8_t> &robotID) const;
};

} /* namespace robot */
} /* namespace msl */
