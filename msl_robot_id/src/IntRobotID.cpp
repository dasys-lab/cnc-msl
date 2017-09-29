#include "msl/robot/IntRobotID.h"

namespace msl
{
namespace robot
{

IntRobotID::IntRobotID(uint8_t *idBytes, int idSize)
    : id(0)
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

uint8_t* IntRobotID::getRaw() const
{
	return (uint8_t*)&this->id;
}

int IntRobotID::getSize() const
{
	return sizeof(int);
};

bool IntRobotID::operator==(const IRobotID &other) const
{
    return this->id == dynamic_cast<const IntRobotID&>(other).id;
}

bool IntRobotID::operator!=(const IRobotID &other) const
{
    return this->id != dynamic_cast<const IntRobotID&>(other).id;
}

bool IntRobotID::operator<(const IRobotID &other) const
{
    return this->id < dynamic_cast<const IntRobotID&>(other).id;
}

bool IntRobotID::operator> (const IRobotID& other) const
{
	return this->id > dynamic_cast<const IntRobotID&>(other).id;
}

std::vector<uint8_t > IntRobotID::toByteVector() const {

	std::vector<uint8_t> bytes;

    for (int i = 0; i < this->getSize(); i++)
    {
    	bytes.push_back(*this->getRaw() + i);
    }

    return bytes;
}

} /* namespace robot */
} /* namespace msl */
