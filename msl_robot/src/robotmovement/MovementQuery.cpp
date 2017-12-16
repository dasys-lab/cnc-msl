#include "msl_robot/robotmovement/MovementQuery.h"
#include "msl_robot/MSLRobot.h"
#include "msl_robot/kicker/Kicker.h"
#include <MSLWorldModel.h>
#include <pathplanner/PathPlannerQuery.h>
#include <pathplanner/evaluator/IPathEvaluator.h>
#include "RawSensorData.h"

namespace msl
{
	MovementQuery::MovementQuery()
	{
		this->egoAlignPoint = nullptr;
		this->egoDestinationPoint = nullptr;
		this->additionalPoints = nullptr;
		this->blockOppPenaltyArea = false;
		this->blockOppGoalArea = false;
		this->blockOwnPenaltyArea = false;
		this->blockOwnGoalArea = false;
		this->block3MetersAroundBall = false;
		this->snapDistance = 0;
		this->angleTolerance = 0;
		this->alloTeamMatePosition = nullptr;
		this->wm = MSLWorldModel::get();

		this->rotateAroundTheBall = false;
		this->circleRadius = -1;
		this->circleCenterPoint = nullptr;
		this->rectangleUpperLeftCorner = nullptr;
		this->rectangleLowerRightCorner = nullptr;
		this->pathEval = nullptr;

		this->velocityMode = VelocityMode::DEFAULT;

	}

	MovementQuery::~MovementQuery()
	{
	}

	shared_ptr<PathPlannerQuery> MovementQuery::getPathPlannerQuery()
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
		this->egoAlignPoint = nullptr;
		this->egoDestinationPoint = nullptr;
		this->additionalPoints = nullptr;
		this->blockOppPenaltyArea = false;
		this->blockOppGoalArea = false;
		this->blockOwnPenaltyArea = false;
		this->blockOwnGoalArea = false;
		this->block3MetersAroundBall = false;
		this->snapDistance = 0;
		this->angleTolerance = 0;
		this->alloTeamMatePosition = nullptr;
		this->wm = MSLWorldModel::get();

	}

	void MovementQuery::blockCircle(shared_ptr<geometry::CNPoint2D> centerPoint, double radius)
	{
		this->circleCenterPoint = centerPoint;
		this->circleRadius = radius;
	}

	void MovementQuery::blockRectangle(shared_ptr<geometry::CNPoint2D> upLeftCorner,
										shared_ptr<geometry::CNPoint2D> lowRightCorner)
	{
		this->rectangleUpperLeftCorner = upLeftCorner;
		this->rectangleLowerRightCorner = lowRightCorner;
	}
}
