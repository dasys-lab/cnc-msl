#pragma once

#include <supplementary/IAgentID.h>

#include <iostream>

namespace msl
{
namespace robot
{

class IntRobotID : public supplementary::IAgentID
{
	friend struct std::hash<msl::robot::IntRobotID>;
  public:
    IntRobotID(const uint8_t* idBytes, int idSize);
    virtual ~IntRobotID();

    uint8_t* getRaw() const;
    int getSize() const;

    std::string toString() const;

    bool operator== ( const supplementary::IAgentID& obj ) const;
    bool operator!= ( const supplementary::IAgentID& obj ) const;
    bool operator< (const supplementary::IAgentID& other) const;
    bool operator> (const supplementary::IAgentID& other) const;

    std::vector<uint8_t> toByteVector() const;
    int getId() const;

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
            return std::hash<int>()(pa.id);
        }
    };
}


