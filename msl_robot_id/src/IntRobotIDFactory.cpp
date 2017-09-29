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

const alica::IRobotID* IntRobotIDFactory::create(std::vector<uint8_t * > &robotID) const
{
	unsigned char *_robotRosID = reinterpret_cast<unsigned char *>(robotID.data());
	return new IntRobotID(_robotRosID, robotID.size());
}



} /* namespace robot */
} /* namespace msl */
