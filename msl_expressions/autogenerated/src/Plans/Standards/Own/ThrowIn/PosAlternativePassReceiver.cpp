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
using geometry::CNPointEgo;
using geometry::CNPointAllo;

/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1461674942156) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    PosAlternativePassReceiver::PosAlternativePassReceiver() :
            DomainBehaviour("PosAlternativePassReceiver")
    {
        /*PROTECTED REGION ID(con1461674942156) ENABLED START*/ //Add additional options here
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
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto egoBallPos = wm->ball->getPositionEgo();
        if (!ownPos || !egoBallPos)
        {
            auto alloBall = egoBallPos->toAllo(*ownPos);
            // Create additional points for path planning
            auto additionalPoints = nonstd::make_optional<vector<geometry::CNPointAllo>>();
            // add alloBall to path planning
            additionalPoints->push_back(alloBall);
            if (!oldBallPos)
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
                auto ids = parent->getAssignment()->getRobotsWorking(ep);
                geometry::CNPointAllo receiverPos;
                // exactly one robot is receiver
                if (ids->size() > 0 && ids->at(0) != -1)
                {
                    // get receiver position by id
                    auto pos = wm->robots->teammates.getTeammatePositionBuffer(ids->at(0)).getLastValidContent();
                    if (pos)
                    {
                        receiverPos.x = pos->x;
                        receiverPos.y = pos->y;
                    }
                    else
                    {
                        cout << "PAPR: no receiver found looking at (0, 0)!" << endl;
                        receiverPos.x = 0;
                        receiverPos.y = 0;
                    }
                }
                msl_actuator_msgs::MotionControl mc;
                CNPointAllo alloTarget;
                CNPointEgo egoTarget;
                // if there is a receiver, align to it

                // calculate target 60cm away from the ball and on a line with the receiver
                if (receiverPos.y > 0)
                {
                    alloTarget.x = receiverPos.x;
                    alloTarget.y = -receiverPos.y + 500;
                    egoTarget = alloTarget.toEgo(*ownPos);
                }
                else
                {
                    alloTarget.x = receiverPos.x;
                    alloTarget.y = -receiverPos.y - 500;
                    egoTarget = alloTarget.toEgo(*ownPos);
                }
                // ask the path planner how to get there
//            mc = msl::RobotMovement::moveToPointCarefully(egoTarget, receiverPos->alloToEgo(*ownPos), 0,
//                                                          additionalPoints);
                query.egoDestinationPoint = egoTarget;
                query.egoAlignPoint = receiverPos.toEgo(*ownPos);
                query.additionalPoints = additionalPoints;
                mc = rm.moveToPoint(query);

                // if we reach the point and are aligned, the behavior is successful
                if (egoTarget.length() < 250 && fabs(egoBallPos->rotateZ(M_PI).angleZ()) < (M_PI / 180) * 5)
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
    }

    void PosAlternativePassReceiver::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1461674942156) ENABLED START*/ //Add additional options here
        string tmp;
        bool success = true;
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
