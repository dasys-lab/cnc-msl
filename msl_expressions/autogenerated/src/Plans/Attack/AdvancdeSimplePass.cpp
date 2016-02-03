using namespace std;
#include "Plans/Attack/AdvancdeSimplePass.h"

/*PROTECTED REGION ID(inccpp1450176193656) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1450176193656) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    AdvancdeSimplePass::AdvancdeSimplePass() :
            DomainBehaviour("AdvancdeSimplePass")
    {
        /*PROTECTED REGION ID(con1450176193656) ENABLED START*/ //Add additional options here
        maxVel = 2000;
        minDistToMate = 2000;
        gotMessage = false;
        sc = supplementary::SystemConfig::getInstance();
        teamMateTaskName = "";
        itcounter = 0;
        receiver = nullptr;
        shared_ptr < geometry::CNPosition > oldMatePos = nullptr;
        /*PROTECTED REGION END*/
    }
    AdvancdeSimplePass::~AdvancdeSimplePass()
    {
        /*PROTECTED REGION ID(dcon1450176193656) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void AdvancdeSimplePass::run(void* msg)
    {
        /*PROTECTED REGION ID(run1450176193656) ENABLED START*/ //Add additional options here
        if (itcounter < 3)
        {
            itcounter++;
            return; //give the ball a few milliseconds to leave the kicker
        }
        msl_actuator_msgs::MotionControl mc;
        shared_ptr < geometry::CNPoint2D > ballPos = wm->ball.getEgoBallPosition();
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision(); //Corrected;
        if (ballPos == nullptr)
        {
            return;
        }
//		if (ownPos == nullptr)
//		{
//
//			mc = DriveHelper.DriveRandomly(1000, WM);
//			send(mc);
//			return;
//		}
//
//		geometry::CNPoint2D egoMatePos = null;
//		Position matePos = null;
//
//		if (receiver != null)
//		{
//			ICollection<int> robots = RobotsInEntryPoint(receiver);
//
//			foreach (int rob in robots)
//			{
//				matePos = SHWM.GetRobotDataByID(rob).PlayerPosition;
//				break;
//			}
//			if (matePos != null)
//			{
//				egoMatePos = WorldHelper.Allo2Ego(matePos.Point, ownPos);
//				oldMatePos = matePos;
//			}
//			else
//			{
//				if (oldMatePos != null)
//					egoMatePos = WorldHelper.Allo2Ego(oldMatePos.Point, ownPos);
//			}
//		}
//
//		if (egoMatePos != null)
//		{
//			mc = PassHelper.MoveToFreeSpace(WorldHelper.Ego2Allo(egoMatePos, ownPos), maxVel, WM);
//			Send(mc);
//		}
        /*PROTECTED REGION END*/
    }
    void AdvancdeSimplePass::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1450176193656) ENABLED START*/ //Add additional options here
        maxVel = (*this->sc)["Behaviour"]->get<double>("Behaviour", "MaxSpeed", NULL);
        oldMatePos = nullptr;
        //planName = parameters["Plan"];
        receiver = getParentEntryPoint(teamMateTaskName);
        itcounter = 0;
        bool success = true;
        try
        {
            success &= getParameter("AttackTask", teamMateTaskName);
        }

        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1450176193656) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
