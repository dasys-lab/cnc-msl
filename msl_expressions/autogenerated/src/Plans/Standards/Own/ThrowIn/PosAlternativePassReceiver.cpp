using namespace std;
#include "Plans/Standards/Own/ThrowIn/PosAlternativePassReceiver.h"

/*PROTECTED REGION ID(inccpp1461674942156) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "SystemConfig.h"
#include "engine/model/EntryPoint.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <Robots.h>

#include <msl/robot/IntRobotID.h>
#include <supplementary/IAgentID.h>
#include <supplementary/BroadcastID.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1461674942156) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    PosAlternativePassReceiver::PosAlternativePassReceiver() :
            DomainBehaviour("PosAlternativePassReceiver")
    {
        /*PROTECTED REGION ID(con1461674942156) ENABLED START*/ //Add additional options here
        query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    PosAlternativePassReceiver::~PosAlternativePassReceiver()
    {
        /*PROTECTED REGION ID(dcon1461674942156) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void PosAlternativePassReceiver::run(void* msg)
    {
        /*PROTECTED REGION ID(run1461674942156) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();
        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }
        shared_ptr < geometry::CNPoint2D > alloBall = egoBallPos->egoToAllo(*ownPos);
        // Create additional points for path planning
        shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
                vector<shared_ptr<geometry::CNPoint2D>>>();
        // add alloBall to path planning
        additionalPoints->push_back(alloBall);
        if (oldBallPos == nullptr)
        {
            oldBallPos = alloBall;
        }
        EntryPoint* ep = getParentEntryPoint(taskName);
        if (ep != nullptr)
        {
            // get the plan in which the behavior is running
            auto parent = this->runningPlan->getParent().lock();
            if (parent == nullptr)
            {
                cout << "parent null" << endl;
                return;
            }
            // get robot ids of robots in found entry point
            shared_ptr<vector<const supplementary::IAgentID*>> ids = parent->getAssignment()->getRobotsWorking(ep);
            shared_ptr < geometry::CNPoint2D > receiverPos = nullptr;
            // exactly one robot is receiver
            if (ids->size() > 0 && !dynamic_cast<const supplementary::BroadcastID*>(ids->at(0)))
            {
                // get receiver position by id
                auto pos = wm->robots->teammates.getTeamMatePosition(dynamic_cast<const msl::robot::IntRobotID*>(ids->at(0)));
                if (pos != nullptr)
                {
                    receiverPos = make_shared < geometry::CNPoint2D > (pos->x, pos->y);
                }
                else
                {
                    cout << "PAPR: no receiver found looking at (0, 0)!" << endl;
                    receiverPos = make_shared < geometry::CNPoint2D > (0, 0);
                }
            }
            msl_actuator_msgs::MotionControl mc;
            shared_ptr < geometry::CNPoint2D > alloTarget = make_shared<geometry::CNPoint2D>();
            shared_ptr < geometry::CNPoint2D > egoTarget = nullptr;
            // if there is a receiver, align to it

            // calculate target 60cm away from the ball and on a line with the receiver
            if (receiverPos->y > 0)
            {
                alloTarget->x = receiverPos->x;
                alloTarget->y = -receiverPos->y + 500;
                egoTarget = alloTarget->alloToEgo(*ownPos);
            }
            else
            {
                alloTarget->x = receiverPos->x;
                alloTarget->y = -receiverPos->y - 500;
                egoTarget = alloTarget->alloToEgo(*ownPos);
            }
            // ask the path planner how to get there
//            mc = msl::RobotMovement::moveToPointCarefully(egoTarget, receiverPos->alloToEgo(*ownPos), 0,
//                                                          additionalPoints);
            query->egoDestinationPoint = egoTarget;
            query->egoAlignPoint = receiverPos->alloToEgo(*ownPos);
            query->additionalPoints = additionalPoints;
            mc = rm.moveToPoint(query);

            // if we reach the point and are aligned, the behavior is successful
            if (egoTarget->length() < 250 && fabs(egoBallPos->rotate(M_PI)->angleTo()) < (M_PI / 180) * 5)
            {
                this->setSuccess(true);
            }
            if (!std::isnan(mc.motion.translation))
            {
                send(mc);
            }
            else
            {
                cout << "Motion command is NaN!" << endl;
            }
        }
        /*PROTECTED REGION END*/
    }
    void PosAlternativePassReceiver::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1461674942156) ENABLED START*/ //Add additional options here
        string tmp;
        bool success = true;
        alloTarget = make_shared < geometry::CNPoint2D > (0, 0);
        oldBallPos.reset();
        oldAlloTarget.reset();
        try
        {
            success &= getParameter("TeamMateTaskName", tmp);
            if (success)
            {
                taskName = tmp;
            }
        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "PAPR: Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1461674942156) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
