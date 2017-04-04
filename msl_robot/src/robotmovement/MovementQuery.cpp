/*
 * MovementQuery.cpp
 *
 *  Created on: Apr 27, 2016
 *      Author: Carpe Noctem
 */

#include "msl_robot/robotmovement/MovementQuery.h"
#include "msl_robot/MSLRobot.h"
#include "msl_robot/kicker/Kicker.h"
#include <MSLWorldModel.h>
#include <pathplanner/PathPlannerQuery.h>

#include <cnc_geometry/Calculator.h>

using nonstd::nullopt;

namespace msl
{

MovementQuery::MovementQuery()
{
    this->reset();
}

MovementQuery::~MovementQuery()
{
}

void MovementQuery::reset()
{
    this->rotateAroundTheBall = false;
    this->circleRadius = -1;
    this->circleCenterPoint = nullopt;
    this->rectangleUpperLeftCorner = nullopt;
    this->rectangleLowerRightCorner = nullopt;
    this->pathEval = nullptr;

    this->egoAlignPoint = nullopt;
    this->egoDestinationPoint = nullopt;
    this->additionalPoints = nullopt;
    this->fast = false;
    this->dribble = false;
    this->blockOppPenaltyArea = false;
    this->blockOppGoalArea = false;
    this->blockOwnPenaltyArea = false;
    this->blockOwnGoalArea = false;
    this->block3MetersAroundBall = false;
    this->snapDistance = 0;
    this->angleTolerance = 0;
    this->alloTeamMatePosition = nullopt;
    this->wm = MSLWorldModel::get();

    resetAllPIDParameters();
    readConfigParameters();
}

shared_ptr<PathPlannerQuery> MovementQuery::getPathPlannerQuery() const
{
    auto ret = make_shared<PathPlannerQuery>();
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

double MovementQuery::rotationPDForDribble(geometry::CNPointEgo egoTarget)
{
    cout << "MovementQuery::rotationPDForDribble: egoTarget = " << egoTarget.toString();

    double angleErr = egoTarget.rotateZ(this->robot->kicker->kickerAngle).angleZ();
    double rot = this->pRot * angleErr +
                 this->dRot * geometry::normalizeAngle(angleErr - this->lastRotDribbleErr); // Rotation PD

    // limit rotation acceleration
    if (rot > this->curRotDribble)
    {
        rot = min(rot, this->curRotDribble + this->rotAccStep);
    }
    else
    {
        rot = max(rot, this->curRotDribble - this->rotAccStep);
    }

    // clamp rotation
    rot = min(abs(rot), this->maxRot) * (rot > 0 ? 1 : -1);

    this->curRotDribble = rot;

    this->lastRotDribbleErr = angleErr;
    return rot;
}

double MovementQuery::translationPIForDribble(double transOrt)
{
    double maxCurTrans = this->maxVel;
    double transErr = abs(this->lastRotDribbleErr);
    if (transErr > this->angleDeadBand)
    {
        this->transControlIntegralDribble += this->iTrans * transErr;
        this->transControlIntegralDribble = min(this->transControlIntegralMax, this->transControlIntegralDribble);
    }
    else
    {
        this->transControlIntegralDribble = 0; // Math.Max(0,transControlIntegral-Math.PI*5);
        transErr = 0;
    }
    maxCurTrans -= this->pTrans * transErr + this->transControlIntegralDribble;
    maxCurTrans = max(0.0, maxCurTrans);

    double transTowards = sqrt(maxCurTrans * maxCurTrans - transOrt * transOrt);
    if (std::isnan(transTowards) || transTowards < 50)
        transTowards = 50;

    if (transTowards > this->curTransDribble)
    {
        transTowards = min(transTowards, this->curTransDribble + this->transAccStep);
    }
    else
    {
        transTowards = max(transTowards, this->curTransDribble - this->transDecStep);
    }

    this->curTransDribble = transTowards;

    return sqrt(transTowards * transTowards + transOrt * transOrt);
}

double MovementQuery::angleCalcForDribble(double transOrt)
{
    auto ballPos = this->wm->ball->getEgoBallPosition();
    auto dir = ballPos->normalize();
    auto ort = geometry::CNPointEgo(dir.y, -dir.x);
    dir = dir * this->curTransDribble + ort * transOrt;
    return dir.angleZ();
}

void MovementQuery::blockCircle(geometry::CNPointAllo centerPoint, double radius)
{
    this->circleCenterPoint = centerPoint;
    this->circleRadius = radius;
}

void MovementQuery::blockRectangle(geometry::CNPointAllo upLeftCorner, geometry::CNPointAllo lowRightCorner)
{
    this->rectangleUpperLeftCorner = upLeftCorner;
    this->rectangleLowerRightCorner = lowRightCorner;
}

/**
 * Reset all Parameters for the methods rotationPDForDribble() and  translationPIForDribble()
 */
void MovementQuery::resetAllPIDParameters()
{
    resetRotationPDParameters();
    resetTransaltionPIParameters();
}

void MovementQuery::resetRotationPDParameters()
{
    this->curRotDribble = 0;
    this->lastRotDribbleErr = 0;
    readConfigParameters();
}

void MovementQuery::resetTransaltionPIParameters()
{
    this->curTransDribble = 0;
    this->transControlIntegralDribble = 0;
    readConfigParameters();
}

/**
 * Sets P and D parameters for rotationPDForDribble()
 * @pParam
 * @dParam
 */
void MovementQuery::setRotationPDParameters(double pParam, double dParam)
{
    this->pRot = pParam;
    this->dRot = dParam;
}

/**
 * Sets P and I parameters for translationPIForDribble()
 * @pParam
 * @iParam
 */
void MovementQuery::setTranslationPIParameters(double pParam, double iParam)
{
    this->pTrans = pParam;
    this->iTrans = iParam;
}

/**
 * Reads all necessary parameters from Dribble.conf
 */
void MovementQuery::readConfigParameters()
{
    supplementary::SystemConfig *supplementary = supplementary::SystemConfig::getInstance();
    // load rotation config parameters
    this->pRot = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "pRot", NULL);
    this->dRot = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "dRot", NULL);
    this->rotAccStep = (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>(
        "DribbleWater", "MaxRotationAcceleration", NULL);
    this->maxRot =
        (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "MaxRotation", NULL);

    // load translation config patamerters
    this->transAccStep =
        (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "MaxAcceleration", NULL);
    this->transDecStep =
        (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "MaxDecceleration", NULL);
    this->iTrans =
        (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "iTrans", NULL) / M_PI;
    this->pTrans =
        (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "pTrans", NULL) / M_PI;
    this->transControlIntegralMax =
        (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "maxTransIntegral", NULL);
    this->angleDeadBand =
        (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "angleDeadBand", NULL) /
        180 * M_PI;
    this->maxVel =
        (*supplementary::SystemConfig::getInstance())["Dribble"]->get<double>("DribbleWater", "MaxVelocity", NULL);
}

} /* namespace msl */
