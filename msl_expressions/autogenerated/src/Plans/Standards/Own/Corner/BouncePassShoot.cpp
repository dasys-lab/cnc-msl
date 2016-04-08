using namespace std;
#include "Plans/Standards/Own/Corner/BouncePassShoot.h"

/*PROTECTED REGION ID(inccpp1459357144291) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1459357144291) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    BouncePassShoot::BouncePassShoot() :
            DomainBehaviour("BouncePassShoot")
    {
        /*PROTECTED REGION ID(con1459357144291) ENABLED START*/ //Add additional options here
        ownPos = nullptr;
        egoBallPos = nullptr;
        planName = "";
        teamMateTaskName = "";
        receiver = nullptr;
        counter = 0;
        driveSlowSpeed = 200.0;
        /*PROTECTED REGION END*/
    }
    BouncePassShoot::~BouncePassShoot()
    {
        /*PROTECTED REGION ID(dcon1459357144291) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void BouncePassShoot::run(void* msg)
    {
        /*PROTECTED REGION ID(run1459357144291) ENABLED START*/ //Add additional options here
        ownPos = wm->rawSensorData.getOwnPositionVision(); //WM.OwnPositionCorrected;
        egoBallPos = wm->ball.getEgoBallPosition();
        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;

        if (ownPos == nullptr)
        {
            mc = msl::RobotMovement::driveRandomly(2000.0);
            send(mc);
            return;
        }

        if (egoBallPos == nullptr)
        {
            return;
        }
        auto robots = robotsInEntryPointOfHigherPlan(receiver);
        shared_ptr < geometry::CNPosition > matePos = nullptr;
        for (int rob : *robots)
        {
            matePos = wm->robots.teammates.getTeamMatePosition(rob);
            break;
        }
        auto egoMatePos = matePos->getPoint()->alloToEgo(*ownPos);
        auto centerOppGoal = make_shared < geometry::CNPoint2D > (wm->field.getFieldLength() / 2, 0);

        //this might only need to be WorldHelper.HaveBall
        if (!wm->ball.haveBallDribble(false))
        {
            mc = msl::RobotMovement::driveToPointAlignNoAvoidance(egoBallPos, egoMatePos, driveSlowSpeed, true);
            bhc.leftMotor = 80;
            bhc.rightMotor = 80;
            send(bhc);
            send(mc);
        }
        else
        {
            counter++;
            mc.motion.angle = 0;
            mc.motion.rotation = 0;
            mc.motion.translation = 0;
            bhc.leftMotor = 0;
            bhc.rightMotor = 0;
            send(bhc);
            send(mc);
        }

        msl_actuator_msgs::KickControl kc;
        //double totalDistance = egoMatePos.Distance() + matePos.DistanceTo(centerOppGoal);
        kc.power = 2800; //(ushort)KickHelper.GetKickPowerPass(totalDistance);
        //kc.Kick.Power*=1.2; //potential loss in energy from ball bouncing, check if this should be included or not
        kc.kicker = wm->kicker.kickerAngle;
        if (wm->ball.haveBall() && counter >= 3)
        {
            kc.enabled = true;
            send(kc);
            this->success = true;
        }
        /*PROTECTED REGION END*/
    }
    void BouncePassShoot::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1459357144291) ENABLED START*/ //Add additional options here
        counter = 0;
        string tmp;
        string tmp2;
        bool success = true;
        success &= getParameter("planName", tmp);
        try
        {
            if (success)
            {
                this->planName = tmp;
            }
            success &= getParameter("teamMateTaskName", tmp2);
            if (success)
            {
                this->teamMateTaskName = tmp2;
                receiver = getHigherEntryPoint(planName, teamMateTaskName);
            }
        }
        catch (exception& e)
        {
            cerr << "S4PP: Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "S4PP: Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1459357144291) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
