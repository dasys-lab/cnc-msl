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

    uint8_t* getRaw() const;
    int getSize() const;

    friend std::ostream& operator<<(std::ostream& os, const msl::robot::IntRobotID& obj)
    {
        os << obj.id;
        return os;
    }
    bool operator== ( const IRobotID& obj ) const;
    bool operator!= ( const IRobotID& obj ) const;
    bool operator< (const IRobotID& other) const;
    bool operator> (const IRobotID& other) const;


  private:
    int id;
};
} /* namespace robot */
} /* namespace msl */


namespace std
{
    template<>
    struct hash<msl::robot::IntRobotID>
    {
        typedef const msl::robot::IntRobotID& argument_type;
        typedef std::size_t result_type;

        result_type operator()(argument_type & pa) const
        {
            return std::hash<int>(pa.id);
        }
    };
}


