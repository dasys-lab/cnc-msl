using namespace std;
#include "Plans/Attack/DribbleAttackConservative.h"

/*PROTECTED REGION ID(inccpp1457967322925) ENABLED START*/ //Add additional includes here
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
        alloGoalMid = wm->field->posOppGoalMid();
        before = false;
        this->setTrigger(wm->getVisionDataEventTrigger());
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
        msl::RobotMovement rm;
        ;
        auto ballPos = wm->ball->getPositionEgo();

        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();

        if (!ownPos)
        {
            return;
        }

        auto goalMid = alloGoalMid.toEgo(*ownPos);
        auto corner = wm->obstacles->getBiggestFreeGoalAreaMidPoint();
        msl_actuator_msgs::MotionControl bm;
        query.egoDestinationPoint = goalMid;
        query.dribble = true;

        auto tmpMC = rm.moveToPoint(query);

        if (!corner && tmpMC.motion.translation != NAN)
        {
            bm = tmpMC;
        }
        else
        {
            query.egoDestinationPoint = corner;
            query.dribble = true;

            auto tmpMC = rm.moveToPoint(query);

            if (tmpMC.motion.translation != NAN)
            {
                corner = nonstd::make_optional<geometry::CNPointAllo>(
                        corner->toAllo(*ownPos) + geometry::CNPointAllo(-800, 0).toEgo(*ownPos));
                bm = tmpMC;
            }
        }

        //if I drive into the enemy goal area
        msl_actuator_msgs::MotionControl mc = rm.ruleActionForBallGetter();
        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        else if (!std::isnan(bm.motion.translation))
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
