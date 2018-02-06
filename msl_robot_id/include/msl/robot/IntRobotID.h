#pragma once

#include <supplementary/IAgentID.h>

#include <iostream>

namespace msl
{
namespace robot
{

class IntRobotIDFactory;
class IntRobotID : public supplementary::IAgentID
{
	friend msl::robot::IntRobotIDFactory;

  public:
    virtual ~IntRobotID();

    uint8_t* getRaw() const;
    int getSize() const;
    uint8_t getType() const;
    std::string toString() const;
    std::size_t hash() const;

    bool operator== (const supplementary::IAgentID& obj) const;
    bool operator!= (const supplementary::IAgentID& obj) const;
    bool operator< (const supplementary::IAgentID& other) const;
    bool operator> (const supplementary::IAgentID& other) const;

    std::vector<uint8_t> toByteVector() const;
    int getId() const;

    static const uint8_t TYPE = 0;

  private:
	IntRobotID(const uint8_t* idBytes, int idSize);
    int id;
};
} /* namespace robot */
} /* namespace msl */
