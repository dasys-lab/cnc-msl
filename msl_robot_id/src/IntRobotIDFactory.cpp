#include "msl/robot/IntRobotIDFactory.h"

#include <engine/IRobotID.h>


namespace msl
{
namespace robot
{

IntRobotIDFactory::IntRobotIDFactory()
{
}

IntRobotIDFactory::~IntRobotIDFactory()
{
}

alica::IRobotID IntRobotIDFactory::create(uint8_t *idBytes, int idSize) const
{
	return IntRobotID(idBytes, idSize);
}

} /* namespace robot */
} /* namespace msl */
