using namespace std;
#include "Plans/Attack/Wander.h"

/*PROTECTED REGION ID(inccpp1434716215423) ENABLED START*/ //Add additional includes here
#include "MSLFootballField.h"
#include "msl_robot/robotmovement/RobotMovement.h"
#include <RawSensorData.h>
#include <Game.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1434716215423) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Wander::Wander() :
            DomainBehaviour("Wander")
    {
        /*PROTECTED REGION ID(con1434716215423) ENABLED START*/ //Add additional options here
        fieldLength = wm->field->getFieldLength();
        fieldWidth = wm->field->getFieldWidth();
        distToCorner = (2500.0 / 18000.0) * fieldLength;
        distToOutLine = (3000.0 / 12000.0) * fieldWidth;
        firstTargetSet = false;
        EPSILON_RADIUS = 1000.0;
        SLOW_DOWN_DISTANCE = 1200.0;
        maxTranslation = 4000.0;
        translation = 0.0;
        currentTargetPoint = geometry::CNPointAllo (0, 0);
        //default targetpoints
        targetPoints.resize(5);
        targetPoints[0] = geometry::CNPointAllo (-(fieldLength / 2 - distToCorner), fieldWidth / 2 - distToCorner);
        targetPoints[1] = geometry::CNPointAllo (-(fieldLength / 2 - distToCorner), -(fieldWidth / 2 - distToCorner));
        targetPoints[2] = geometry::CNPointAllo (0.0, 0.0);
        targetPoints[3] = geometry::CNPointAllo (fieldLength / 2 - distToCorner, fieldWidth / 2 - distToCorner);
        targetPoints[4] = geometry::CNPointAllo (fieldLength / 2 - distToCorner, -(fieldWidth / 2 - distToCorner));

        //drive into both opps corners
        targetPointsOwnCorner.resize(2);
        targetPointsOwnCorner[0] = geometry::CNPointAllo (fieldLength / 2 - distToCorner, fieldWidth / 2 - distToCorner);
        targetPointsOwnCorner[1] = geometry::CNPointAllo (fieldLength / 2 - distToCorner, -(fieldWidth / 2 - distToCorner));

        //drive into both own corners
        targetPointsOppCorner.resize(2);
        targetPointsOppCorner[0] = geometry::CNPointAllo (-(fieldLength / 2 - distToCorner), fieldWidth / 2 - distToCorner);
        targetPointsOppCorner[1] = geometry::CNPointAllo (-(fieldLength / 2 - distToCorner), -(fieldWidth / 2 - distToCorner));

        //drive to both own penaltyarea corners
        targetPointsOwnGoalKick.resize(2);
        targetPointsOwnGoalKick[0] = geometry::CNPointAllo (-(fieldLength / 2 - wm->field->getGoalAreaLength()), wm->field->getGoalAreaWidth() / 2);
        targetPointsOwnGoalKick[1] = geometry::CNPointAllo(-(fieldLength / 2 - wm->field->getGoalAreaLength()), -wm->field->getGoalAreaWidth() / 2);

        //drive to both opp penaltyarea corners
        targetPointsOppGoalKick.resize(2);
        targetPointsOppGoalKick[0] = geometry::CNPointAllo (fieldLength / 2 - wm->field->getGoalAreaLength(), wm->field->getGoalAreaWidth() / 2);
        targetPointsOppGoalKick[1] = geometry::CNPointAllo (fieldLength / 2 - wm->field->getGoalAreaLength(), -wm->field->getGoalAreaWidth() / 2);

        //points on side lines (distToOutline away)
        targetPointsThrowIn.resize(6);
        //left
        targetPointsThrowIn[0] = geometry::CNPointAllo (fieldLength / 3, fieldWidth / 2 - distToOutLine);
        targetPointsThrowIn[1] = geometry::CNPointAllo (0.0, fieldWidth / 2 - distToOutLine);
        targetPointsThrowIn[2] = geometry::CNPointAllo (-fieldLength / 3, fieldWidth / 2 - distToOutLine);
        //right
        targetPointsThrowIn[3] = geometry::CNPointAllo (fieldLength / 3, -(fieldWidth / 2 - distToOutLine));
        targetPointsThrowIn[4] = geometry::CNPointAllo (0.0, -(fieldWidth / 2 - distToOutLine));
        targetPointsThrowIn[5] = geometry::CNPointAllo (-fieldLength / 3, -(fieldWidth / 2 - distToOutLine));

        /*PROTECTED REGION END*/
    }
    Wander::~Wander()
    {
        /*PROTECTED REGION ID(dcon1434716215423) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Wander::run(void* msg)
    {
        /*PROTECTED REGION ID(run1434716215423) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;

        auto ownPosition = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        if (!ownPosition)
            return;
        msl::Situation situation = wm->game->getSituation();
        if (!firstTargetSet)
        {
            setFirstTargetPoint(situation);
            firstTargetSet = true;
        }
        auto currentEgoTarget = currentTargetPoint.toEgo(*ownPosition);

        if (currentEgoTarget.length() < EPSILON_RADIUS)
        {
            if (situation == msl::Situation::OwnCorner)
            {
                currentTargetPoint = targetPointsOwnCorner[rand() % targetPointsOwnCorner.size()];
            }
            else if (situation == msl::Situation::OppCorner)
            {
                currentTargetPoint = targetPointsOppCorner[rand() % targetPointsOppCorner.size()];
            }
            else if (situation == msl::Situation::OwnGoalkick)
            {
                currentTargetPoint = targetPointsOwnGoalKick[rand() % targetPointsOwnGoalKick.size()];
            }
            else if (situation == msl::Situation::OppGoalkick)
            {
                currentTargetPoint = targetPointsOppGoalKick[rand() % targetPointsOppGoalKick.size()];
            }

            else if (situation == msl::Situation::OwnThrowin || situation == msl::Situation::OppThrowin)
            {
                currentTargetPoint = targetPointsThrowIn[rand() % targetPointsThrowIn.size()];
            }
            else
            {
                currentTargetPoint = targetPoints[rand() % targetPoints.size()];
            }
        }

        auto targetPoint = currentTargetPoint.toEgo(*ownPosition);

        double targetDistance = targetPoint.length();

        translation = maxTranslation;

        if (targetDistance < SLOW_DOWN_DISTANCE)
        {
            translation = targetDistance;
        }

        // replaced with new moveToPoint method
//        msl_actuator_msgs::MotionControl mc = msl::RobotMovement::moveToPointCarefully(targetPoint, targetPoint, 0);
        query.egoDestinationPoint = targetPoint;
        query.egoAlignPoint = targetPoint;

        msl_actuator_msgs::MotionControl mc = rm.moveToPoint(query);
        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        else
        {
            cout << "motion commant is NaN" << endl;
        }

        /*PROTECTED REGION END*/
    }
    void Wander::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1434716215423) ENABLED START*/ //Add additional options here
        firstTargetSet = false;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1434716215423) ENABLED START*/ //Add additional methods here
    void Wander::setFirstTargetPoint(msl::Situation situation)
    {
        if (situation == msl::Situation::OwnCorner)
        {
            currentTargetPoint = targetPointsOwnCorner[rand() % targetPointsOwnCorner.size()];
        }
        else if (situation == msl::Situation::OppCorner)
        {
            currentTargetPoint = targetPointsOppCorner[rand() % targetPointsOppCorner.size()];
        }
        else if (situation == msl::Situation::OwnGoalkick)
        {
            currentTargetPoint = targetPointsOwnGoalKick[rand() % targetPointsOwnGoalKick.size()];
        }
        else if (situation == msl::Situation::OppGoalkick)
        {
            currentTargetPoint = targetPointsOppGoalKick[rand() % targetPointsOppGoalKick.size()];
        }

        else if (situation == msl::Situation::OwnThrowin || situation == msl::Situation::OppThrowin)
        {
            currentTargetPoint = targetPointsThrowIn[rand() % targetPointsThrowIn.size()];
        }
        else
        {
            currentTargetPoint = targetPoints[rand() % targetPoints.size()];
        }
    }
/*PROTECTED REGION END*/
} /* namespace alica */
