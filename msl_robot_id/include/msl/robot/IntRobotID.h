#pragma once

#include <engine/IRobotID.h>

#include <iostream>

namespace msl
{
namespace robot
{

class IntRobotID : public alica::IRobotID
{
  public:
    IntRobotID(uint8_t* idBytes, int idSize);
    virtual ~IntRobotID();

    friend std::ostream& operator<<(std::ostream& os, const msl::robot::IntRobotID& obj);

    bool operator== ( const IRobotID& obj ) const;

  private:
    int id;
};

std::ostream& operator<<(std::ostream& os, const msl::robot::IntRobotID& obj)
{
    os << obj.id << std::endl;
    return os;
}

} /* namespace robot */
} /* namespace msl */


