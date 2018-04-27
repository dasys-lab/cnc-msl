using namespace std;
#include "Plans/Standards/Own/ThrowIn/PositionAlternativeReceiver.h"

/*PROTECTED REGION ID(inccpp1462978634990) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/robotmovement/MovementQuery.h>
#include "engine/Assignment.h"
#include "engine/RunningPlan.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include <msl_robot/MSLRobot.h>
#include <RawSensorData.h>
#include <Robots.h>
#include <MSLWorldModel.h>
#include <Ball.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1462978634990) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    PositionAlternativeReceiver::PositionAlternativeReceiver() :
            DomainBehaviour("PositionAlternativeReceiver")
    {
        /*PROTECTED REGION ID(con1462978634990) ENABLED START*/ //Add additional options here
        this->query = make_shared<msl::MovementQuery>();
        this->ballDistRec = 0.0;
        this->teamMateTaskName = "";
        /*PROTECTED REGION END*/
    }
    PositionAlternativeReceiver::~PositionAlternativeReceiver()
    {
        /*PROTECTED REGION ID(dcon1462978634990) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void PositionAlternativeReceiver::run(void* msg)
    {
        /*PROTECTED REGION ID(run1462978634990) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();
        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        auto otherRec = this->getTeammatePosFromTaskName(this->teamMateTaskName);
        shared_ptr < geometry::CNPoint2D > alloBall = egoBallPos->egoToAllo(*ownPos);
        // Create additional points for path planning
        shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
                vector<shared_ptr<geometry::CNPoint2D>>>();
        // add alloBall to path planning
        additionalPoints->push_back(alloBall);

        msl_actuator_msgs::MotionControl mc;
        shared_ptr < geometry::CNPoint2D > alloTarget = make_shared<geometry::CNPoint2D>();

        //other receiver makes alignment decision
        if(otherRec != nullptr) {

            if (otherRec->y < 0)
            {
                alloTarget->y = alloBall->y + ballDistRec;
            }
            else
            {
                alloTarget->y = alloBall->y - ballDistRec;
            }

        } else {

            if (alloBall->y < 0)
            {
                alloTarget->y = alloBall->y + ballDistRec;
            }
            else
            {
                alloTarget->y = alloBall->y - ballDistRec;
            }

        }

        alloTarget->x = alloBall->x;

        auto egoTarget = alloTarget->alloToEgo(*ownPos);

        query->egoDestinationPoint = egoTarget;
        query->egoAlignPoint = egoBallPos;
        query->additionalPoints = additionalPoints;

        mc = this->robot->robotMovement->moveToPoint(query);

        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        else
        {
            cout << "Motion command is NaN!" << endl;
        }

        /*PROTECTED REGION END*/
    }
    void PositionAlternativeReceiver::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1462978634990) ENABLED START*/ //Add additional options here
        this->ballDistRec = (*sc)["Drive"]->get<double>("Drive.KickOff.BallDistRec", NULL);
        this->teamMateTaskName = "";

        string tmp;
        bool success = true;
        try
        {
            success &= getParameter("TeamMateTaskName", tmp);
            if (success)
            {
                teamMateTaskName = tmp;
            }

        }
        catch (exception &e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "PAR: Parameter does not exist" << endl;
        }

        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1462978634990) ENABLED START*/ //Add additional methods here
    shared_ptr<geometry::CNPoint2D> PositionAlternativeReceiver::getTeammatePosFromTaskName(string teamMateTaskName)
    {
        shared_ptr < geometry::CNPoint2D > recPos = nullptr;

        EntryPoint *ep = getParentEntryPoint(teamMateTaskName);
        if (ep != nullptr)
        {
            // get the plan in which the behavior is running
            auto parent = this->runningPlan->getParent().lock();
            if (parent != nullptr)
            {
                // get robot ids of robots in found entry point
                shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep);
                // exactly one robot is receiver
                if (ids->size() > 0 && ids->at(0) != -1)
                {
                    // get receiver position by id
                    auto pos = this->wm->robots->teammates.getTeamMatePosition(ids->at(0));
                    if (pos != nullptr)
                    {
                        recPos = pos->getPoint();
                    }
                }
            }
        }

        return recPos;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
