#pragma once

#include "msl_actuator_msgs/MotionControl.h"
#include "msl_robot/robotmovement/MovementQuery.h"

#include <cnc_geometry/CNPointEgo.h>
#include <nonstd/optional.hpp>
//#include <Plans/DribbleCalibration/Container/DribbleCalibrationContainer.h>

//#define DEBUG_MOVE_CONT

namespace msl
{
class MovementContainer
{
  public:
    MovementContainer();
    virtual ~MovementContainer();

    enum Movement
    {
        Forward,
        Backward,
        Left,
        Right,
        ForwardRight,
        ForwardLeft,
        BackwardRight,
        BackwardLeft
    };
    const char *movementToString[8] = {"Forward",       "Backward",     "Left",           "Right",
                                       "Forward right", "Forward left", "Backward right", "Backward left"};

    msl_actuator_msgs::MotionControl getBall();
    msl_actuator_msgs::MotionControl move(Movement movement, int translation);
    bool checkObstacles(Movement movement, double distance);

  private:
    // output variables
    bool changeDirFlag;

    msl::MSLWorldModel *wm;

    msl::MovementQuery query;

    double defaultDistance;
    double distToObs;
    bool changeDirections;
    bool rotateAroundTheBall;
    double angleTolerance;
    nonstd::optional<geometry::CNPointAllo> alloAlignPoint;

    /**
     * @movement describes the movement direction
     * @distance to the returned ego destination point
     *
     * @return an ego destination point depending on the movement direction
     */
    nonstd::optional<geometry::CNPointEgo> getEgoDestinationPoint(Movement movement, double distance);
    nonstd::optional<geometry::CNPointEgo> calcNewAlignPoint(Movement curMove);
    bool checkFieldLines(geometry::CNPointEgo egoDest);
    msl_actuator_msgs::MotionControl setZero(msl_actuator_msgs::MotionControl mc);
    msl_actuator_msgs::MotionControl setNaN(msl_actuator_msgs::MotionControl mc);
    Movement getNewDirection(int curDir, vector<Movement> movement, int next);
    void readOwnConfigParameter();
};
}
