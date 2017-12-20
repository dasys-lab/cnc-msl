#include "msl_robot/MSLRobot.h"
#include "msl_robot/kicker/Kicker.h"
#include "msl_robot/robotmovement/RobotMovement.h"
#include <MSLWorldModel.h>

namespace msl
{
MSLRobot *MSLRobot::get()
{
    static MSLRobot instance;
    return &instance;
}

MSLRobot::MSLRobot()
{
    this->wm = MSLWorldModel::get();
    this->robotMovement = RobotMovement::get();
    this->kicker = new Kicker(wm);
}

MSLRobot::~MSLRobot()
{
    delete this->kicker;
}

} /* namespace msl */
