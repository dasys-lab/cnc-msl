#pragma once

#include "IntRobotID.h"

#include <engine/IRobotID.h>
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

    const alica::IRobotID* create (uint8_t* idBytes, int idSize) const;
};

} /* namespace robot */
} /* namespace msl */
