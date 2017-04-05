/*
 * MovementContainer.cpp
 *
 *  Created on: Dec 14, 2016
 *      Author: cn
 */

#include <Ball.h>
#include <MSLFootballField.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <SystemConfig.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_robot/dribbleCalibration/container/MovementContainer.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <obstaclehandler/Obstacles.h>
#include <pathplanner/PathPlanner.h>

using msl_actuator_msgs::MotionControl;
using nonstd::optional;
using nonstd::nullopt;

namespace msl
{
MovementContainer::MovementContainer()
{
    distToObs = 0;
    rotateAroundTheBall = false;
    angleTolerance = 0;
    changeDirections = 0;
    wm = msl::MSLWorldModel::get();
    defaultDistance = 0;
    changeDirFlag = false;
    readOwnConfigParameter();
}

MovementContainer::~MovementContainer()
{
    // TODO Auto-generated destructor stub
}

MotionControl MovementContainer::getBall()
{
    msl::RobotMovement rm;
    query.reset();
    msl_actuator_msgs::MotionControl mc;

    auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
    if (!ownPos)
    {
        cerr << "MovementContainer::getBall() -> couldn't get vision data" << endl;
    }

    auto egoBallPos = wm->ball->getAlloBallPosition()->toEgo(*ownPos);
    query.egoDestinationPoint = egoBallPos;
    query.egoAlignPoint = egoBallPos;

    mc = rm.moveToPoint(query);
    mc.motion.translation += 400;
    return mc;
}

/**
 * movement may be: FORWARD, BACKWARD, LEFT, RIGHT
 *
 * @return MotionControl to drive in a direction while using PathPlaner and avoid obstacles
 */
msl_actuator_msgs::MotionControl MovementContainer::move(Movement movement, int translation)
{
    msl_actuator_msgs::MotionControl mc;
    msl::RobotMovement rm;

    if (movement != Forward && movement != Backward && movement != Left && movement != Right)
    {
        cerr << "MovementContainer::move() -> invalid input parameter" << endl;
        return setNaN(mc);
    }

    // for output
    if (changeDirFlag)
    {
        cout << "changeDirections... " << endl;
        changeDirFlag = false;
    }

    // check if there is an obstacle
    if (!this->changeDirections && checkObstacles(movement, defaultDistance))
    {
        this->changeDirections = true;
        changeDirFlag = true;
    }

    // if there is an obstacle or we are in front of a field line, change directions
    if (changeDirections)
    {
        auto me = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();

        if (this->alloAlignPoint)
        {
            alloAlignPoint = calcNewAlignPoint(movement)->toAllo(*me);
        }

        geometry::CNPointEgo newAlignPoint = alloAlignPoint->toEgo(*me);

#ifdef DEBUG_MOVE_CONT
        cout << "MC::move() newAlignPoint: " << newAlignPoint->toString() << endl;
        cout << "MC::move() newALignPoint->angleTo() = " << fabs(newAlignPoint->angleTo()) << endl;
#endif

        if (fabs(newAlignPoint.angleZ()) < (M_PI - query.angleTolerance))
        {

#ifdef DEBUG_MOVE_CONT
            cout << "rotateAroundTheBall = " << (rotateAroundTheBall ? "true" : "false") << endl;
            cout << "angleTolerance = " << angleTolerance << endl;
#endif
            query.egoAlignPoint = newAlignPoint;
            query.rotateAroundTheBall = rotateAroundTheBall;
            query.angleTolerance = angleTolerance;

            cout << "query->egoAlignPoint = " << query.egoAlignPoint->toString();
            mc = rm.alignTo(query);

#ifdef DEBUG_MOVE_CONT
            cout << "MC::move() changeDirections: " << (changeDirections ? "true" : "false")
                 << " -> rotating to new align point" << endl;
            cout << "MC::move() MotionControl translation = " << mc.motion.translation << endl;
            cout << "MC::move() MotionControl rotation = " << mc.motion.rotation << endl;
            cout << "MC::move() MotionControl angle = " << mc.motion.angle << endl;
#endif

            return mc;
        }
        else
        {
            alloAlignPoint = nullopt;
            changeDirections = false;
            return setZero(mc);
        }
    }
    else
    {
        query.reset();

        auto egoDestination = getEgoDestinationPoint(movement, defaultDistance);
        query.egoDestinationPoint = egoDestination;

#ifdef DEBUG_MOVE_CONT
        cout << "MC::move() egoDestinationPoint = " << egoDestination->toString();
#endif
        mc = rm.moveToPoint(query);
        mc.motion.translation = translation;
#ifdef DEBUG_MOVE_CONT
        cout << "MC::move() changeDirections: " << (changeDirections ? "true" : "false") << " drive normally" << endl;
#endif
        return mc;
    }
    cerr << "MovementContainer::move() MotionControl mc = NaN!" << endl;
    return setNaN(mc);
}

/**
 * @movement describes the direction in witch you will check for obstacles
 *
 * @return true if there is an obstacle in movement direction
 */
bool MovementContainer::checkObstacles(Movement movement, double distance)
{
    if (movement != Forward && movement != Backward && movement != Left && movement != Right &&
        movement != ForwardRight && movement != ForwardLeft && movement != BackwardRight && movement != BackwardLeft)
    {
        cerr << "MovementContainer::checkObstacles() -> wrong input" << endl;
        return true;
    }

    // currenPos 		: CNPoint2D -> ownPos
    auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
    // goal				: CNPoint2D -> destPoint
    auto egoDest = getEgoDestinationPoint(movement, distance);
    // obstacelPoint	: CNPOint2D -> obstacle
    auto obs = wm->obstacles->getRawObstaclesAlloBuffer().getLastValidContent();

    if (egoDest && checkFieldLines(*egoDest))
    {
        // field line in front of me
        return true;
    }

    if (!obs || (*obs)->size() == 0)
    {
        return false;
    }

    if(ownPos && egoDest)
    {
        for (auto &ob : *(*obs))
        {
            return wm->pathPlanner->corridorCheck(ownPos->getPoint(), egoDest->toAllo(*ownPos), ob);
        }
    }

    return false;
}

optional<geometry::CNPointEgo> MovementContainer::calcNewAlignPoint(Movement curMove)
{
#ifdef DEBUG_MOVE_CONT
    cout << "MC::calcNewAlignPoint choose new direction..." << endl;
#endif

    // use checkObstacels to choose a new direction
    vector<Movement> movement = {Forward,  ForwardRight, Right, BackwardRight,
                                 Backward, BackwardLeft, Left,  ForwardLeft};

    double distance = 3000;
    optional<geometry::CNPointEgo> alignPoint;

    for (int i = 0; i < movement.size(); i++)
    {
        Movement dir = movement.at(i);

        if (!checkObstacles(dir, distance))
        {
            {
#ifdef DEBUG_MOVE_CONT
                cout << "MC::calcNewAlignPoint curMove = " << movementToString[curMove] << endl;
                cout << "MC::calcNewAlignPoint New align point in " << movementToString[dir] << " direction..." << endl;
#endif
                dir = curMove == Left ? getNewDirection(i, movement, 4) : dir;
                dir = curMove == Right ? getNewDirection(i, movement, -4) : dir;
#ifdef DEBUG_MOVE_CONT
                cout << "MC::calcNewAlignPoint New specific align point in " << movementToString[dir] << " direction..."
                     << endl;
#endif
                alignPoint = getEgoDestinationPoint(dir, distance);
                break;
            }
        }
    }
    return alignPoint;
}

optional<geometry::CNPointEgo> MovementContainer::getEgoDestinationPoint(Movement movement, double distance)
{
    if (movement != Forward && movement != Backward && movement != Left && movement != Right &&
        movement != ForwardRight && movement != ForwardLeft && movement != BackwardRight && movement != BackwardLeft)
    {
        cout << "MovementContainer::getEgoDestinationPoint -> wrong input!" << endl;
        return nullopt;
    }

    double beta = 45;
    double a = distance * cos(beta);
    double b = sqrt((distance * distance) - (a * a));

    // 1000 = 1m

    switch (movement)
    {
    case Forward:
        return geometry::CNPointEgo(-distance, 0);
    case Backward:
        return geometry::CNPointEgo(distance, 0);
    case Left:
        return geometry::CNPointEgo(0, -distance);
    case Right:
        return geometry::CNPointEgo(0, distance);
    case ForwardRight:
        return geometry::CNPointEgo(-b, a);
    case ForwardLeft:
        return geometry::CNPointEgo(-b, -a);
    case BackwardRight:
        return geometry::CNPointEgo(b, a);
    case BackwardLeft:
        return geometry::CNPointEgo(b, -a);
    }
}

/**
 * @return true if the movement destination is outside the field
 */
bool MovementContainer::checkFieldLines(geometry::CNPointEgo egoDest)
{
#ifdef DEBUG_MOVE_CONT
    cout << "MC::checkFieldLines egoDestination = " << egoDest->toString();
#endif

    auto me = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();

    if(me) {
        // egoDestinationPoint to allo
        geometry::CNPointAllo alloDestination = egoDest.toAllo(*me);

#ifdef DEBUG_MOVE_CONT
        cout << "MC::checkFieldLines alloDestinationPoint = " << alloDestination->toString();
#endif

        // check if destinationPoint is inside the field area
        double fieldLength = wm->field->getFieldLength();
        double fieldWidth = wm->field->getFieldWidth();

        if (fabs(alloDestination.x) > (fieldLength / 2) || fabs(alloDestination.y) > (fieldWidth / 2))
        {
            cout << "Field line in front of me!" << endl;
            return true;
        }
    }

    return false;
}

/**
 * helper function for calcNewAlignPoint
 * uses vector<Movement> movement like a ring list
 *
 * @return the new align point if the robot is driving in another direction than forward
 */
MovementContainer::Movement MovementContainer::getNewDirection(int curDir, vector<Movement> movement, int next)
{
#ifdef DEBUG_MOVE_CONT
    cout << "MC::getNewDirection: curDir = " << curDir << " next = " << next
         << " movement.size = " << (int)movement.size() << endl;
#endif

    if ((curDir - next) >= 0 && (curDir - next) < (int)movement.size())
    {
#ifdef DEBUG_MOVE_CONT
        cout << "MC::getNewDirection: ((curDir - next) >= 0 && (curDir - next) < (int) movement.size()) -> return "
                "movement.at(curDir - next)"
             << endl;
#endif
        return movement.at(curDir - next);
    }
    else if ((curDir - next) > ((int)movement.size() - 1))
    {
        int i = (curDir - next) - ((int)movement.size() - 1);
#ifdef DEBUG_MOVE_CONT
        cout << "MC::getNewDirection: ((curDir - next) > ((int) movement.size() - 1)) i = " << i << endl;
#endif
        return movement.at(i);
    }
    else
    {
        int i = ((int)movement.size() - 1) - abs(curDir - next);
#ifdef DEBUG_MOVE_CONT
        cout << "MC::getNewDirection: i = " << i << " abs(curDir - next) = abs(" << curDir << " - " << next
             << ") = " << abs(curDir - next) << endl;
#endif
        return movement.at(i);
    }
}

MotionControl MovementContainer::setZero(MotionControl mc)
{
    mc.motion.translation = 0;
    mc.motion.angle = 0;
    mc.motion.rotation = 0;
    return mc;
}

MotionControl MovementContainer::setNaN(MotionControl mc)
{
    mc.senderID = -1;
    mc.motion.translation = NAN;
    mc.motion.rotation = NAN;
    mc.motion.angle = NAN;
    return mc;
}

void MovementContainer::readOwnConfigParameter()
{
    supplementary::SystemConfig *sys = supplementary::SystemConfig::getInstance();
    //	robotRadius = (*sys)["Rules"]->get<double( "Rules.RobotRadius", NULL);
    defaultDistance = (*sys)["DribbleCalibration"]->get<double>("DribbleCalibration.Default.DefaultDistance", NULL);
    rotateAroundTheBall =
        (*sys)["DribbleCalibration"]->get<bool>("DribbleCalibration.Default.RotateAroundTheBall", NULL);
    angleTolerance = (*sys)["DribbleCalibration"]->get<double>("DribbleCalibration.Default.AngleTolerance", NULL);
}

} /* namespace msl */
