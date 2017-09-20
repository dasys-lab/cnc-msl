#include "msl/robot/IntRobotID.h"

namespace msl
{
namespace robot
{

IntRobotID::IntRobotID(uint8_t *idBytes, int idSize)
    : IRobotID(idBytes, idSize)
    , id(0)
{
    if (idSize != 4)
    {
        std::cerr << "IntRobotID expects 4 bytes to create an integer!" << std::endl;
        return;
    }

    for (int i = idSize; i > 0; i--)
    {
        this->id += (*idBytes << (idSize * 8));
    }
}

IntRobotID::~IntRobotID()
{
}

bool IntRobotID::operator==(const IRobotID &obj) const
{
    return this->id == dynamic_cast<const IntRobotID&>(obj).id;
}

} /* namespace robot */
} /* namespace msl */
