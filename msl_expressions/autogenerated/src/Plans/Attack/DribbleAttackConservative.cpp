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
    	field = msl::MSLFootballField::getInstance();
	    alloGoalMid = field->posOwnGoalMid();
		before = false;
		this->behaviourTrigger = wm->getVisionDataEventTrigger();
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
		auto ballPos = wm->ball.getEgoBallPosition();
		auto dstscan = wm->rawSensorData.getDistanceScan();

		auto ownPos = wm->rawSensorData.getOwnPositionVision();

		if (ownPos==nullptr) {
			return;
		}

		auto goalMid = alloGoalMid->alloToEgo(*ownPos);
		//MotionControl bm = DribbleHelper.DribbleToPoint(goalMid,translation,WM);

		auto corner = wm->obstacles.getBiggestFreeGoalAreaMidPoint();
		msl_actuator_msgs::MotionControl bm;
		shared_ptr<geometry::CNPoint2D> pathPlanningPoint;
		if( corner == nullptr  && msl::RobotMovement::dribbleToPointConservative(goalMid, pathPlanningPoint) != nullptr) {
				bm = *msl::RobotMovement::dribbleToPointConservative(goalMid, pathPlanningPoint);
		}
		else if(msl::RobotMovement::dribbleToPointConservative(corner, pathPlanningPoint) !=  nullptr){
				corner = (corner->egoToAllo(*ownPos)+ make_shared<geometry::CNPoint2D>(-800,0)->alloToEgo(*ownPos));
//				corner = WorldHelper.Allo2Ego(WorldHelper.Ego2Allo(corner,ownPos) + (new Point2D(-800,0)),ownPos);
//				bm = DribbleHelperNew.DribbleToPoint(corner, out pathPlanningPoint);
				bm = *msl::RobotMovement::dribbleToPointConservative(corner, pathPlanningPoint);
		}


		shared_ptr<geometry::CNPoint2D> turnTo;
		if (ballPos!=nullptr) msl::RobotMovement::dribbleNeedToTurn(ownPos,ballPos,pathPlanningPoint);
		if (turnTo!=nullptr) {
//			HHelper.SetTargetPoint(turnTo); // TODO ?
			this->failure = true;
		}

//		if( bm == nullptr) return;
		//if i drive into the enemy goal area
		bm = msl::RobotMovement::nearGoalArea(bm);

		send(bm);

        /*PROTECTED REGION END*/
    }
    void DribbleAttackConservative::initialiseParameters()
    {
        /*PROTECTED REGION ID(intialiseParameters1457967322925) ENABLED START*/ //Add additional options here

        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1457967322925) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
