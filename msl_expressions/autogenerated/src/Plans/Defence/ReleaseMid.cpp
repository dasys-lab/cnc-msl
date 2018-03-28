using namespace std;
#include "Plans/Defence/ReleaseMid.h"

/*PROTECTED REGION ID(inccpp1458033482289) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <engine/RunningPlan.h>
#include <engine/model/AbstractPlan.h>
#include <Robots.h>
#include <RawSensorData.h>
#include <MSLWorldModel.h>
#include <Ball.h>
#include <Logger.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1458033482289) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    ReleaseMid::ReleaseMid() :
            DomainBehaviour("ReleaseMid")
    {
        /*PROTECTED REGION ID(con1458033482289) ENABLED START*/ //Add additional options here
        teamMateTaskName = "";
        teamMatePlanName = "";
        ep = nullptr;
        teamMateId = 0;
        threshold = 0.0;
        yHysteresis = 0.0;
        vMax = 0.0;
        query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    ReleaseMid::~ReleaseMid()
    {
        /*PROTECTED REGION ID(dcon1458033482289) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void ReleaseMid::run(void* msg)
    {
        /*PROTECTED REGION ID(run1458033482289) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPoint2D > referencePoint = nullptr; // Point we want to align and pos to
        msl_actuator_msgs::MotionControl mc;
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        if (ownPos == nullptr)
        {
            mc = this->robot->robotMovement->driveRandomly(500);
            send(mc);
            //cout << "AAPR: OwnPos is null" << endl;
            this->logger->log(this->getName(), "OnPos is null", msl::LogLevels::error);
            return;
        }

        if (ep == nullptr)
        {
            //cout << this->getRunningPlan()->getPlan()->getName() << ": EP is null" << endl;
            this->logger->log(this->getName(), this->getRunningPlan()->getPlan()->getName() + ": EP is null", msl::LogLevels::debug);
            return;
        }

        // the only teammate in the corresponding task/ entrypoint
        auto teammates = robotsInEntryPointOfHigherPlan(ep);
        if (teammates)
        {
            for (int mateId : *teammates)
            {
                this->teamMateId = mateId;
                break;
            }
        }

        // determine the best reference point
        if (this->teamMateId != 0)
        { // take the teammate as reference point
            auto teammate = wm->robots->teammates.getTeamMatePosition(teamMateId);
            referencePoint = make_shared < geometry::CNPoint2D > (teammate->x, teammate->y);
        }
        else if (egoBallPos != nullptr)
        { // take the ball as reference point
            referencePoint = egoBallPos->egoToAllo(*ownPos);
        }
        else
        { // no teammate and no ball, hmpf stay inside the middle of the field
            referencePoint = make_shared < geometry::CNPoint2D > (4000.0, 0.0);
        }

        shared_ptr < geometry::CNPoint2D > targetPoint = make_shared<geometry::CNPoint2D>(); // point we want to drive to
        targetPoint->x = min(max(0.0, referencePoint->x - 2000.0), 3000.0); // because of 0.0, this behaviour should be triggered
        targetPoint->y = (referencePoint->y * 2) / 3;
        if (abs(targetPoint->y - referencePoint->y) < 500.0)
        {
            if (referencePoint->y > threshold)
            {
                threshold = -yHysteresis;
                targetPoint->y = referencePoint->y - 500.0;
            }
            else
            {
                threshold = +yHysteresis;
                targetPoint->y = referencePoint->y + 500.0;
            }
        }
        // repaced moveToPointCarefully with new moveToPoint method
        query->egoDestinationPoint = targetPoint->alloToEgo(*ownPos);
        query->snapDistance = 50;
        if (egoBallPos != nullptr)
        {
//            mc = msl::RobotMovement::moveToPointCarefully(targetPoint->alloToEgo(*ownPos), egoBallPos, 50, nullptr);
            query->egoAlignPoint = egoBallPos;
            mc = this->robot->robotMovement->moveToPoint(query);
        }
        else
        {
//            mc = msl::RobotMovement::moveToPointCarefully(targetPoint->alloToEgo(*ownPos),
//                                                          referencePoint->alloToEgo(*ownPos), 50, nullptr);
            query->egoAlignPoint = referencePoint->alloToEgo(*ownPos);
            mc = this->robot->robotMovement->moveToPoint(query);
        }
        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        /*PROTECTED REGION END*/
    }
    void ReleaseMid::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1458033482289) ENABLED START*/ //Add additional options here
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        vMax = (*this->sc)["Behaviour"]->get<double>("Behaviour", "MaxSpeed", NULL);
        bool success = true;
        try
        {
            string tmp = "";
            success &= getParameter("YHysteresis", tmp);
            if (success)
            {
                this->yHysteresis = stod(tmp);
            }
            success &= getParameter("TeamMatePlanName", teamMatePlanName);
            success &= getParameter("TeamMateTaskName", teamMateTaskName);
        }
        catch (exception& e)
        {
            //cerr << "Could not cast the parameter properly" << endl;
            this->logger->log(this->getName(), "Could not cast parameter properly", msl::LogLevels::error);
        }
        if (!success)
        {
            //cerr << "StandardAlignAndGrab: Parameter does not exist" << endl;
        	this->logger->log(this->getName(), "Parameter does not exist", msl::LogLevels::error);
        }

        ep = getHigherEntryPoint(teamMatePlanName, teamMateTaskName);
        if (ep == nullptr)
        {
            //cerr << "ReleaseMid: Receiver==null, because planName, teamMateTaskName does not match" << endl;
            this->logger->log(this->getName(), "ReleaseMid: Receiver==null, because planName, teamMateTaskName does not match", msl::LogLevels::debug);
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1458033482289) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
