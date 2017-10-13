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

    const IntRobotID* create(const std::vector<uint8_t> &robotID) const;
    const IntRobotID* create(int robotID) const;
    const IntRobotID* generateID() const;

  private:

};

} /* namespace robot */
} /* namespace msl */
