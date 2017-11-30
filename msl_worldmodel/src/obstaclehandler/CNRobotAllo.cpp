#include "obstaclehandler/CNRobotAllo.h"

#include "cnc_geometry/CNPositionAllo.h"
#include "obstaclehandler/CNRobotEgo.h"

using std::make_shared;
using std::vector;
using std::string;
using std::endl;

namespace msl
{

CNRobotAllo::CNRobotAllo()
    : velocity()
{
    this->radius = 0;
    this->id = 0;
    this->certainty = 0;
    this->rotationVel = 0;
    this->opposer = make_shared<vector<int>>();
    this->supporter = make_shared<vector<int>>();
}

CNRobotAllo::~CNRobotAllo()
{
}

string CNRobotAllo::toString() const
{
    std::stringstream ss;
    ss << "CNRobot: ID: " << this->id << " Pos: " << this->position << " Velocity: " << this->velocity << endl;
    return ss.str();
}

CNRobotEgo CNRobotAllo::toEgo(geometry::CNPositionAllo &ownPos)
{
    auto robotEgo = CNRobotEgo();

    robotEgo.radius = this->radius;
    robotEgo.id = this->id;
    robotEgo.certainty = this->certainty;
    robotEgo.rotationVel = this->rotationVel;
    robotEgo.opposer = make_shared<vector<int>>(*this->opposer);
    robotEgo.supporter = make_shared<vector<int>>(*this->supporter);
    robotEgo.position = this->position.toEgo(ownPos);
    robotEgo.velocity = this->velocity.toEgo(ownPos);

    return robotEgo;
}

} /* namespace msl */
