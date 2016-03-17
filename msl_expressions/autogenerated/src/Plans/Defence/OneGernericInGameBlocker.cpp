using namespace std;
#include "Plans/Defence/OneGernericInGameBlocker.h"

/*PROTECTED REGION ID(inccpp1458034268108) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "engine/RunningPlan.h"
#include "engine/model/AbstractPlan.h"
#include "SolverType.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1458034268108) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    OneGernericInGameBlocker::OneGernericInGameBlocker() :
            DomainBehaviour("OneGernericInGameBlocker")
    {
        /*PROTECTED REGION ID(con1458034268108) ENABLED START*/ //Add additional options here
        query = make_shared < ConstraintQuery > (wm->getEngine());
        maxVel = 0.0;
        avoidBall = false;
        lastResultFound = 0;
        failTimeThreshold = 0;
        teamMateTaskName = "";
        teamMatePlanName = "";
        ep = nullptr;
        teamMateId = 0;
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
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > ballPos = wm->ball.getEgoBallPosition();
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
            cout << "OGIGB Taskname " << teamMateTaskName << " Planname " << teamMatePlanName << ": EP is null" << endl;
            return;
        }
        // the only teammate in the corresponding task/ entrypoint
        auto teammates = robotsInEntryPointOfHigherPlan(ep);
        for (int mateId : *teammates)
        {
            this->teamMateId = mateId;
            break;
        }
        shared_ptr < geometry::CNPosition > attackerPos = nullptr;
        // determine the best reference point
        if (this->teamMateId != 0)
        { // take the teammate as reference point
            attackerPos = wm->robots.teammates.getTeamMatePosition(teamMateId);
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
        cout << "BEH " << this->getRunningPlan()->getPlan()->getName() << ": Solver found valid solution: " << ret
                << endl;

        if (ret)
        {
            lastResultFound = wm->getTime();
        }
        else
        {
            if (wm->getTime() - lastResultFound > failTimeThreshold)
            {
                this->success = true;
            }
            mc.motion.angle = 0;
            mc.motion.translation = 0;
            mc.motion.rotation = 0;
            send(mc);
            return;
        }
        if (result.size() > 1)
        {
            shared_ptr < geometry::CNPoint2D > driveTo = make_shared < geometry::CNPoint2D
                    > (result.at(0), result.at(1))->alloToEgo(*ownPos);

            shared_ptr < geometry::CNPoint2D > ownRelativePos = ownPos->getPoint()->alloToEgo(*attackerPos);
            shared_ptr < geometry::CNPoint2D > relativeGoalPos = driveTo->alloToEgo(*attackerPos);

            if (relativeGoalPos->x < -250 && ownRelativePos->length() > relativeGoalPos->length() + 1000)
            {
                this->success = true;
                return;
            }

            if (avoidBall)
            {
                mc = msl::RobotMovement::placeRobotCareBall(driveTo, ballPos, maxVel);
            }
            else
            {
                mc = msl::RobotMovement::placeRobotAggressive(driveTo, ballPos, maxVel);
            }
        }
        send(mc);
        /*PROTECTED REGION END*/
    }
    void OneGernericInGameBlocker::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1458034268108) ENABLED START*/ //Add additional options here
        query->clearStaticVariables();
        query->addVariable(getVariablesByName("X"));
        query->addVariable(getVariablesByName("Y"));
        result.clear();
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        maxVel = (*this->sc)["Behaviour"]->get<double>("Behaviour", "MaxSpeed", NULL);
        bool success = true;
        try
        {
            string tmp = "";
            success &= getParameter("FailTimeTreshold", tmp);
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
            cout << teamMatePlanName << " : " << teamMateTaskName << endl;
        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
            avoidBall = false;
            failTimeThreshold = 250000000;
        }
        if (!success)
        {
            cerr << "OneGenericInGameBlocker: Parameter does not exist" << endl;
        }

        ep = getHigherEntryPoint(teamMatePlanName, teamMateTaskName);
        if (ep == nullptr)
        {
            cerr << "OneGenericInGameBlocker: Receiver==null, because planName, teamMateTaskName does not match"
                    << endl;
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1458034268108) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
