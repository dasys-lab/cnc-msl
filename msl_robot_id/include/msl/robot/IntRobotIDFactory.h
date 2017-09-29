#pragma once

#include "IntRobotID.h"

#include <engine/IRobotID.h>
#include <engine/IRobotIDFactory.h>

#include <vector>

namespace msl
{
namespace robot
{

class IntRobotIDFactory : public alica::IRobotIDFactory
{
  public:
    IntRobotIDFactory();
    virtual ~IntRobotIDFactory();

    const alica::IRobotID* create(std::vector<uint8_t> &robotID) const;
};

} /* namespace robot */
} /* namespace msl */
