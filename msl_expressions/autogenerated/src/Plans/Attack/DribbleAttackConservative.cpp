using namespace std;
#include "Plans/Attack/DribbleAttackConservative.h"

/*PROTECTED REGION ID(inccpp1457967322925) ENABLED START*/ //Add additional includes here
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
        //CorrectedOdometryData odom = WM.OdometryData;
        auto ballPos = wm->ball->getEgoBallPosition();
//        auto dstscan = wm->rawSensorData.getDistanceScan();

        auto ownPos = wm->rawSensorData->getOwnPositionVision();

        if (ownPos == nullptr)
        {
            return;
        }

        auto goalMid = alloGoalMid->alloToEgo(*ownPos);
        //MotionControl bm = DribbleHelper.DribbleToPoint(goalMid,translation,WM);
        auto corner = wm->obstacles->getBiggestFreeGoalAreaMidPoint();
        msl_actuator_msgs::MotionControl bm;
        shared_ptr < geometry::CNPoint2D > pathPlanningPoint = make_shared<geometry::CNPoint2D>();
        auto tmpMC = msl::RobotMovement::dribbleToPointConservative(goalMid, pathPlanningPoint);

        if (corner == nullptr && tmpMC != nullptr)
        {
            bm = *tmpMC;
        }
        else
        {
            tmpMC = msl::RobotMovement::dribbleToPointConservative(corner, pathPlanningPoint);
            if (tmpMC != nullptr)
            {
                corner =
                        (corner->egoToAllo(*ownPos) + make_shared < geometry::CNPoint2D > (-800, 0)->alloToEgo(*ownPos));
                bm = *tmpMC;
            }
        }

//        shared_ptr < geometry::CNPoint2D > turnTo;
//        if (ballPos != nullptr)
//            msl::RobotMovement::dribbleNeedToTurn(ownPos, ballPos, pathPlanningPoint);
//        if (turnTo != nullptr)
//        {
//			HHelper.SetTargetPoint(turnTo); // TODO ?
//            this->failure = true;
//        }

        //if i drive into the enemy goal area
        bm = msl::RobotMovement::nearGoalArea(bm);

        send(bm);

        /*PROTECTED REGION END*/
    }
    void DribbleAttackConservative::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1457967322925) ENABLED START*/ //Add additional options here
        msl::RobotMovement::reset();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1457967322925) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
