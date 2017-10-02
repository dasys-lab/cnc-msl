#include "msl/robot/IntRobotIDFactory.h"

#include <supplementary/IAgentID.h>

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

const supplementary::IAgentID* IntRobotIDFactory::create(std::vector<uint8_t> &robotID) const
{
	unsigned char *_robotRosID = reinterpret_cast<unsigned char *>(robotID.data());
	return new IntRobotID(_robotRosID, robotID.size());
}



} /* namespace robot */
} /* namespace msl */
