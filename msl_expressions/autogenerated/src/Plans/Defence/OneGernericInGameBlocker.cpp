using namespace std;
#include "Plans/Defence/OneGernericInGameBlocker.h"

/*PROTECTED REGION ID(inccpp1458034268108) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <engine/RunningPlan.h>
#include <engine/model/AbstractPlan.h>
#include <SolverType.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <Robots.h>
#include <msl_helper_msgs/DebugMsg.h>
#include <MSLWorldModel.h>
#include <MSLEnums.h>
#include <Logger.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1458034268108) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    OneGernericInGameBlocker::OneGernericInGameBlocker() :
            DomainBehaviour("OneGernericInGameBlocker")
    {
        /*PROTECTED REGION ID(con1458034268108) ENABLED START*/ //Add additional options here
        query = make_shared < Query > (wm->getEngine());
        maxVel = 0.0;
        avoidBall = false;
        lastResultFound = 0;
        failTimeThreshold = 0;
        teamMateTaskName = "";
        teamMatePlanName = "";
        ep = nullptr;
        teamMateId = 0;
        movQuery = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    OneGernericInGameBlocker::~OneGernericInGameBlocker()
    {
        /*PROTECTED REGION ID(dcon1458034268108) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void OneGernericInGameBlocker::run(void* msg)
    {
        /*PROTECTED REGION ID(run1458034268108) ENABLED START*/ //Add additional options here
        msl_actuator_msgs::MotionControl mc;
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > ballPos = wm->ball->getEgoBallPosition();
        if (ownPos == nullptr || ballPos == nullptr)
        {
            mc.motion.angle = 0;
            mc.motion.translation = 0;
            mc.motion.rotation = 0;
            send(mc);
            return;
        }

        if (ep == nullptr)
        {
            //cout << "OGIGB Taskname " << teamMateTaskName << " Planname " << teamMatePlanName << ": EP is null" << endl;
            this->logger->log(this->getName(), "Taskname " + teamMateTaskName + " Planname " + teamMatePlanName + ": EP is null", msl::LogLevels::error);
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
        shared_ptr < geometry::CNPosition > attackerPos = nullptr;
        // determine the best reference point
        if (this->teamMateId != 0)
        { // take the teammate as reference point
            attackerPos = wm->robots->teammates.getTeamMatePosition(teamMateId);
        }

        if (attackerPos == nullptr)
        {
            mc.motion.angle = 0;
            mc.motion.translation = 0;
            mc.motion.rotation = 0;

            send(mc);
            return;
        }

        bool ret = query->getSolution(SolverType::GRADIENTSOLVER, runningPlan, result);
        std::string tv = "false";
        if(ret)
        {
        	tv = "true";
        }
        //cout << "BEH " << this->getRunningPlan()->getPlan()->getName() << ": Solver found valid solution: " << (ret ? "true" : "false") << endl;
        this->logger->log(this->getName(), "BEH " + this->getRunningPlan()->getPlan()->getName() + ": Solver found valid solution: " + tv, msl::LogLevels::debug);

        if (false == ret)
        {
            if (wm->getTime() - lastResultFound > failTimeThreshold)
            {
                this->setSuccess(true);
            }
            mc.motion.angle = 0;
            mc.motion.translation = 0;
            mc.motion.rotation = 0;
            send(mc);

            return;
        }

        lastResultFound = wm->getTime();

        auto driveTo = make_shared < geometry::CNPoint2D > (result.at(0), result.at(1));

        auto ownRelativePos = ownPos->getPoint()->alloToEgo(*attackerPos);
        auto relativeGoalPos = driveTo->alloToEgo(*attackerPos);

        if (relativeGoalPos->x < -250 && // block only targets which are in driving direction of attacker
                ownRelativePos->length() > relativeGoalPos->length() + 1000) // i'm more than 1 meter away from the blocking position, i failed, better set status to success
        {
            this->setSuccess(true);
            return;
        }

        // Sending debug message for visualization
        msl_helper_msgs::DebugMsg debugMsg;
        debugMsg.topic = "OneGenericInGameBlocker";
        debugMsg.senderID = this->getOwnId();
        debugMsg.validFor = 2000000000;

        msl_helper_msgs::DebugPoint point;

        point.radius = 0.12;
        point.point.x = driveTo->x;
        point.point.y = driveTo->y;
        point.red = 0;
        point.green = 0;
        point.blue = 255;

        debugMsg.points.push_back(point);
        this->send(debugMsg);
        // ---------------------------------------

        // ego centric
        driveTo = driveTo->alloToEgo(*ownPos);

        if (avoidBall)
        {
            // replaced with new moveToPoint method
//            mc = msl::RobotMovement::placeRobotCareBall(driveTo, ballPos, maxVel);

            movQuery->egoDestinationPoint = driveTo;
            movQuery->egoAlignPoint = ballPos;
            mc = this->robot->robotMovement->moveToPoint(movQuery);
            if (driveTo->length() < 100)
            {
                mc.motion.translation = 0;
            }

        }
        else
        {
            // replaced method with new moveToPoint method
//            mc = msl::RobotMovement::placeRobotAggressive(driveTo, ballPos, maxVel);
            movQuery->egoDestinationPoint = driveTo;
            movQuery->egoAlignPoint = ballPos;
            movQuery->velocityMode = msl::VelocityMode::FAST;
            mc = this->robot->robotMovement->moveToPoint(movQuery);
        }

        send(mc);
        /*PROTECTED REGION END*/
    }
    void OneGernericInGameBlocker::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1458034268108) ENABLED START*/ //Add additional options here
        this->lastResultFound = wm->getTime();
        query->clearStaticVariables();
        query->addStaticVariable(getVariablesByName("X"));
        query->addStaticVariable(getVariablesByName("Y"));
        result.clear();
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        maxVel = (*this->sc)["Behaviour"]->get<double>("Behaviour", "MaxSpeed", NULL);
        bool success = true;
        try
        {
            string tmp = "";
            success &= getParameter("FailTimeTresholdMS", tmp);
            if (success)
            {
                this->failTimeThreshold = stol(tmp) * 1000000;
            }
            success &= getParameter("AvoidBall", tmp);
            if (success)
            {
                std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
                istringstream(tmp) >> std::boolalpha >> avoidBall;
            }

            success &= getParameter("TeamMatePlanName", teamMatePlanName);
            success &= getParameter("TeamMateTaskName", teamMateTaskName);
//            cout << teamMatePlanName << " : " << teamMateTaskName << endl;
            this->logger->log(this->getName(), teamMatePlanName + " : " + teamMateTaskName, msl::LogLevels::debug);
        }
        catch (exception& e)
        {
            //cerr << "Could not cast the parameter properly" << endl;
            this->logger->log(this->getName(), "Could not cast parameter properly", msl::LogLevels::error);
            avoidBall = false;
            failTimeThreshold = 250000000;
        }

        if (!success)
        {
            //cerr << "OneGenericInGameBlocker: Parameter does not exist" << endl;
        	this->logger->log(this->getName(), "Parameter does not exist", msl::LogLevels::error);
        }

        ep = getHigherEntryPoint(teamMatePlanName, teamMateTaskName);
        if (ep == nullptr)
        {
            //cerr << "OneGenericInGameBlocker: Receiver==null, because planName, teamMateTaskName does not match" << endl;
            this->logger->log(this->getName(), "Receiver==null, because planName, teamMateTaskName does not match", msl::LogLevels::error);
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1458034268108) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
