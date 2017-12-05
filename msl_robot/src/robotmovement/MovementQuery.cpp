#include "msl_robot/robotmovement/MovementQuery.h"
#include "msl_robot/MSLRobot.h"
#include "msl_robot/kicker/Kicker.h"
#include <MSLEnums.h>
#include <RawSensorData.h>
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

    this->velocityMode = VelocityMode::DEFAULT;
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
}
