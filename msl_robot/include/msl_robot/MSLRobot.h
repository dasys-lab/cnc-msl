#pragma once

namespace msl
{
class MSLWorldModel;
class RobotMovement;
class Kicker;
class MSLRobot
{
  public:
    static MSLRobot *get();

    MSLWorldModel *wm;
    RobotMovement *robotMovement;
    Kicker *kicker;

  private:
    MSLRobot();
    virtual ~MSLRobot();
};

} /* namespace msl */

