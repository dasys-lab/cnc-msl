#pragma once

#include <engine/IRobotIDFactory.h>

namespace msl
{
namespace robot
{

class IntRobotIDFactory : public alica::IRobotIDFactory
{
  public:
    IntRobotIDFactory();
    virtual ~IntRobotIDFactory();
};

} /* namespace robot */
} /* namespace msl */
