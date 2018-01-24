using namespace std;
#include "Plans/Attack/DribbleAttackConservative.h"

/*PROTECTED REGION ID(inccpp1457967322925) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <obstaclehandler/Obstacles.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1457967322925) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DribbleAttackConservative::DribbleAttackConservative() :
            DomainBehaviour("DribbleAttackConservative")
    {
        /*PROTECTED REGION ID(con1457967322925) ENABLED START*/ //Add additional options here
        before = false;
//        this->setTrigger(wm->getVisionDataEventTrigger());
        query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    DribbleAttackConservative::~DribbleAttackConservative()
    {
        /*PROTECTED REGION ID(dcon1457967322925) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DribbleAttackConservative::run(void* msg)
    {
        /*PROTECTED REGION ID(run1457967322925) ENABLED START*/ //Add additional options here
        auto ownPos = wm->rawSensorData->getOwnPositionVision();

        if (ownPos == nullptr )
        {
            return;
        }
        auto tmp = make_shared<geometry::CNPoint2D>(wm->field->posOppGoalMid()->x - wm->field->getPenaltyAreaLength(), wm->field->posOppGoalMid()->y);
        auto tragetPoint = tmp->alloToEgo(*ownPos);
        auto corner = wm->obstacles->getBiggestFreeGoalAreaMidPoint();
        msl_actuator_msgs::MotionControl bm;
        query->egoDestinationPoint = tragetPoint;
        query->egoAlignPoint = tragetPoint;

        auto tmpMC = this->robot->robotMovement->moveToPoint(query);

        if (corner == nullptr && tmpMC.motion.translation != NAN)
        {
            bm = tmpMC;
        }
        else
        {
            query->egoDestinationPoint = corner;

            auto tmpMC = this->robot->robotMovement->moveToPoint(query);

            if (tmpMC.motion.translation != NAN)
            {
                corner =
                        (corner->egoToAllo(*ownPos) + make_shared < geometry::CNPoint2D > (-800, 0)->alloToEgo(*ownPos));
                bm = tmpMC;
            }
        }

        //if I drive into the enemy goal area
        msl_actuator_msgs::MotionControl mc = this->robot->robotMovement->ruleActionForBallGetter();
        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        else
        {
            send(bm);
        }

        /*PROTECTED REGION END*/
    }
    void DribbleAttackConservative::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1457967322925) ENABLED START*/ //Add additional options here
//        msl::RobotMovement::reset();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1457967322925) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
