/*
 * MovementQuery.cpp
 *
 *  Created on: Apr 27, 2016
 *      Author: Carpe Noctem
 */

#include "msl_robot/robotmovement/MovementQuery.h"
#include "RawSensorData.h"
#include "msl_robot/MSLRobot.h"
#include "msl_robot/kicker/Kicker.h"
#include <MSLWorldModel.h>
#include <pathplanner/PathPlannerQuery.h>
#include <pathplanner/evaluator/IPathEvaluator.h>

using nonstd::make_optional;
using nonstd::nullopt;

namespace msl
{
MovementQuery::MovementQuery()
{
    this->egoAlignPoint = nullopt;
    this->egoDestinationPoint = nullopt;
    this->additionalPoints = nullopt;
    this->blockOppPenaltyArea = false;
    this->blockOppGoalArea = false;
    this->blockOwnPenaltyArea = false;
    this->blockOwnGoalArea = false;
    this->block3MetersAroundBall = false;
    this->snapDistance = 0;
    this->angleTolerance = 0;
    this->alloTeamMatePosition = nullopt;
    this->wm = MSLWorldModel::get();

    this->rotateAroundTheBall = false;
    this->circleRadius = -1;
    this->circleCenterPoint = nullopt;
    this->rectangleUpperLeftCorner = nullopt;
    this->rectangleLowerRightCorner = nullopt;
    this->pathEval = nullptr;

    this->velocityMode = Velocity::DEFAULT;

    readConfigParameters();
}

MovementQuery::~MovementQuery()
{
}

shared_ptr<PathPlannerQuery> MovementQuery::getPathPlannerQuery() const
{
    shared_ptr<PathPlannerQuery> ret = make_shared<PathPlannerQuery>();
    ret->additionalPoints = this->additionalPoints;
    ret->block3MetersAroundBall = this->block3MetersAroundBall;
    ret->blockOppGoalArea = this->blockOppGoalArea;
    ret->blockOppPenaltyArea = this->blockOppPenaltyArea;
    ret->blockOwnGoalArea = this->blockOwnGoalArea;
    ret->blockOwnPenaltyArea = this->blockOwnPenaltyArea;
    ret->circleCenterPoint = this->circleCenterPoint;
    ret->circleRadius = this->circleRadius;
    ret->rectangleLowerRightCorner = this->rectangleLowerRightCorner;
    ret->rectangleUpperLeftCorner = this->rectangleUpperLeftCorner;
    return ret;
}

/**
 * PT-Controller for smooth translation acceleration
 */
std::valarray<double> MovementQuery::ptController(double rotation, double translation)
{
    double input[] = {translation, rotation};

    pastControlInput.push(std::valarray<double>(input, 2));

    // slope variable
    controllerVelocity = defaultControllerVelocity;
    if (velocityMode == Velocity::FAST)
    {
        controllerVelocity = fastControllerVelocity;
    }
    else if (velocityMode == Velocity::CAREFULLY)
    {
        controllerVelocity = carefullyControllerVelocity;
    }

    // 0.15 is fix and may not be changed -> fastest acceleration without overshoot
    translation = translation * 0.15 * controllerVelocity;
    rotation = rotation * 0.15 * controllerVelocity;

    // changing point for slope
    double b = pow(controllerVelocity, 2.0);
    // sending frequency
    double TA = 1.0 / 30.0;

    double n1 = 1.0 - exp(-controllerVelocity * TA) - exp(-controllerVelocity * TA) * controllerVelocity * TA;
    double n2 = exp(-2 * controllerVelocity * TA) - exp(-controllerVelocity * TA) + exp(-controllerVelocity * TA) * TA * controllerVelocity;

    double d1 = -2 * exp(-controllerVelocity * TA);
    double d2 = exp(-2 * controllerVelocity * TA);

    //		cout << "n1 = " << n1 << endl;
    //		cout << "n2 = " << n2 << endl;
    //
    //		cout << "d1 = " << d1 << endl;
    //		cout << "d2 = " << d2 << endl;

    pastTranslations.push(std::valarray<double>(init, 2));
    pastTranslations.back() += n2 * pastControlInput.front() - d2 * pastTranslations.front();
    pastControlInput.pop();
    pastTranslations.pop();
    pastTranslations.back() += n1 * pastControlInput.front() - d1 * pastTranslations.front();

    return pastTranslations.back();
}

void MovementQuery::reset()
{
    this->egoAlignPoint = nullopt;
    this->egoDestinationPoint = nullopt;
    this->additionalPoints = nullopt;
    this->blockOppPenaltyArea = false;
    this->blockOppGoalArea = false;
    this->blockOwnPenaltyArea = false;
    this->blockOwnGoalArea = false;
    this->block3MetersAroundBall = false;
    this->snapDistance = 0;
    this->angleTolerance = 0;
    this->alloTeamMatePosition = nullopt;
    this->wm = MSLWorldModel::get();

    readConfigParameters();
}

void MovementQuery::blockCircle(geometry::CNPointAllo centerPoint, double radius)
{
    this->circleCenterPoint = make_optional<geometry::CNPointAllo>(centerPoint);
    this->circleRadius = radius;
}

void MovementQuery::blockRectangle(geometry::CNPointAllo upLeftCorner, geometry::CNPointAllo lowRightCorner)
{
    this->rectangleUpperLeftCorner = make_optional<geometry::CNPointAllo>(upLeftCorner);
    this->rectangleLowerRightCorner = make_optional<geometry::CNPointAllo>(lowRightCorner);
}

/**
 * Initialize all needed parameters and queues for the PT-Controller
 */
void MovementQuery::initializePTControllerParameters()
{
    // initial pt-controller stuff
    std::queue<std::valarray<double>> controlInput;

    nonstd::optional<msl_msgs::MotionInfo> odom;
    if (wm->isUsingSimulator())
    {
        odom = wm->rawSensorData->getOwnVelocityVisionBuffer().getLastValidContent();
    }
    else
    {
        odom = wm->rawSensorData->getOwnVelocityMotionBuffer().getLastValidContent();
    }

    double translation;
    double angle;
    double rotation;

    if (!odom)
    {
        cerr << "MovementQuery: no odometry! Initialize translation, angle and rotation with 0" << endl;
        translation = 0;
        angle = 0;
        rotation = 0;
    }
    else
    {
        auto translation = odom->translation;
        auto angle = odom->angle;
        auto rotation = (double)odom->rotation;
    }

    double input[] = {cos(angle) * translation, sin(angle) * translation, rotation};
    double init[2] = {0.0, 0.0};
    if (pastTranslations.empty())
    {
        pastTranslations.push(std::valarray<double>(init, 2));
        pastTranslations.push(std::valarray<double>(init, 2));
    }
    if (pastControlInput.empty())
    {
        pastControlInput.push(std::valarray<double>(init, 2));
        pastControlInput.push(std::valarray<double>(init, 2));
    }
}

void MovementQuery::clearPTControllerQueues()
{

    while (!pastControlInput.empty())
    {
    	pastControlInput.pop();
    }
    while (!pastTranslations.empty())
    {
    	pastTranslations.pop();
    }
}

/**
 * Reads all necessary parameters from Dribble.conf
 */
void MovementQuery::readConfigParameters()
{
    supplementary::SystemConfig *supplementary = supplementary::SystemConfig::getInstance();
    // load rotation config parameters
    this->carefullyControllerVelocity = (*supplementary)["Drive"]->get<double>("Drive.RobotMovement.PTController.CarefullyControllerVelocity", NULL);
    this->defaultControllerVelocity = (*supplementary)["Drive"]->get<double>("Drive.RobotMovement.PTController.DefaultControllerVelocity", NULL);
    this->fastControllerVelocity = (*supplementary)["Drive"]->get<double>("Drive.RobotMovement.PTController.FastControllerVelocity", NULL);
    this->controllerVelocity = defaultControllerVelocity;
}

void MovementQuery::stopTranslation()
{
	this->pastTranslations.front()[0] = 0.0;
	this->pastTranslations.back()[0] = 0.0;
	this->pastControlInput.front()[0] = 0.0;
	this->pastControlInput.back()[0] = 0.0;
}
}
