using namespace std;
#include "Plans/Attack/AdvancdeSimplePass.h"

/*PROTECTED REGION ID(inccpp1450176193656) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include <Robots.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
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
        this->sc = supplementary::SystemConfig::getInstance();
        teamMateTaskName = "";
        itcounter = 0;
        receiver = nullptr;
        oldMatePos = nonstd::nullopt;
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
        msl::RobotMovement rm;

        if (itcounter < 3)
        {
            itcounter++;
            return; //give the ball a few milliseconds to leave the kicker
        }
        msl_actuator_msgs::MotionControl mc;
        auto ballPos = wm->ball->getPositionEgo();
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent(); //Corrected;
        if (!ballPos)
        {
            return;
        }
        if (!ownPos)
        {

            mc = rm.driveRandomly(1000);
            send(mc);
            return;
        }

        nonstd::optional<geometry::CNPointEgo> egoMatePos = nonstd::nullopt;
        nonstd::optional<geometry::CNPositionAllo> matePos = nonstd::nullopt;

        if (receiver != nullptr)
        {
            auto robots = robotsInEntryPoint(receiver); //make_shared<vector<int>>(); // = robotsInEntryPoint(receiver); // please fix this compile error, greatings Stopfer

            if (robots->size() > 0)
            {
                matePos = wm->robots->teammates.getTeammatePositionBuffer(robots->at(0)).getLastValidContent(); //SHWM.GetRobotDataByID(rob).PlayerPosition;
            }
            if (matePos)
            {
                egoMatePos = nonstd::make_optional<geometry::CNPointEgo>(matePos->getPoint().toEgo(*ownPos));
                oldMatePos = matePos;
            }
            else
            {
                if (oldMatePos)
                {
                    egoMatePos = nonstd::make_optional<geometry::CNPointEgo>(oldMatePos->getPoint().toEgo(*ownPos));
                }
            }
        }

        if (egoMatePos)
        {
            // replaced with new method
//            mc = msl::RobotMovement::moveToFreeSpace(egoMatePos->egoToAllo(*ownPos), maxVel);
            query.alloTeamMatePosition = egoMatePos->toAllo(*ownPos);
            rm.moveToFreeSpace(query);
            send(mc);
        }
        /*PROTECTED REGION END*/
    }
    void AdvancdeSimplePass::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1450176193656) ENABLED START*/ //Add additional options here
        maxVel = (*this->sc)["Behaviour"]->get<double>("Behaviour", "MaxSpeed", NULL);
        oldMatePos = nonstd::nullopt;
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
            cerr << "ASP: Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1450176193656) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
