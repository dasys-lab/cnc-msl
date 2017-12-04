using namespace std;
#include "Plans/Behaviours/PositionExecutor.h"

/*PROTECTED REGION ID(inccpp1438790362133) ENABLED START*/ //Add additional includes here
#include "engine/model/EntryPoint.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"

#include "MSLWorldModel.h"
#include "pathplanner/PathProxy.h"
#include "pathplanner/evaluator/PathEvaluator.h"

#include <RawSensorData.h>
#include <Ball.h>
#include <Robots.h>
#include <Game.h>

using nonstd::optional;
using nonstd::make_optional;
using nonstd::nullopt;

/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1438790362133) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    PositionExecutor::PositionExecutor() :
            DomainBehaviour("PositionExecutor")
    {
        /*PROTECTED REGION ID(con1438790362133) ENABLED START*/ //Add additional options here
        readConfigParameters();
        /*PROTECTED REGION END*/
    }
    PositionExecutor::~PositionExecutor()
    {
        /*PROTECTED REGION ID(dcon1438790362133) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void PositionExecutor::run(void* msg)
    {
        /*PROTECTED REGION ID(run1438790362133) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;

        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent(); // actually ownPosition corrected
        auto egoBallPos = wm->ball->getPositionEgo();

        // return if necessary information is missing
        if (!ownPos || !egoBallPos)
        {
            return;
        }

        // Create allo ball
        auto alloBall = egoBallPos->toAllo(*ownPos);

        // Create additional points for path planning
        auto additionalPoints = make_optional<vector<geometry::CNPointAllo>>();
        // add alloBall to path planning
        additionalPoints->push_back(alloBall);

        // get entry point of task name to locate robot with task name

        if (receiverEp != nullptr)
        {
            // get the plan in which the behavior is running
            auto parent = this->runningPlan->getParent().lock();
            if (parent == nullptr)
            {
                cout << "parent null" << endl;
                return;
            }
            // get robot ids of robots in found entry point
            shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(receiverEp);
            optional<geometry::CNPositionAllo> receiverPos = nullopt;
            // exactly one robot is receiver
            int id = ids->at(0);
            if (id != -1)
            {
                // get receiver position by id
                receiverPos = wm->robots->teammates.getTeammatePositionBuffer(id).getLastValidContent();;
            }
            msl_actuator_msgs::MotionControl mc;
            geometry::CNPointEgo egoTarget;

            if (receiverPos)
            {
                // calculate target 60cm away from the ball and on a line with the receiver
                egoTarget = (alloBall + ((alloBall - receiverPos->getPoint()).normalize() * ballDistanceEx)).toEgo(*ownPos);
            }
            else
            {
                // if there is no receiver, align to middle
                egoTarget = (alloBall + alloTargetVec).toEgo(*ownPos);
            }

            msl::MSLWorldModel* wm = msl::MSLWorldModel::get();
            if (wm->game->getSituation() == msl::Situation::Start)
            { // they already pressed start and we are still positioning, so speed up!
              // removed with new moveToPoint method
//                mc = msl::RobotMovement::moveToPointFast(egoTarget, egoBallPos, fastCatchRadius, additionalPoints);
                query.egoDestinationPoint = egoTarget;
                query.egoAlignPoint = egoBallPos;
                query.snapDistance = fastCatchRadius;
                query.additionalPoints = additionalPoints;
                query.velocityMode = msl::MovementQuery::Velocity::FAST;
                mc = rm.moveToPoint(query);
            }
            else
            { // still enough time to position ...
//                mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoBallPos, slowCatchRadius, additionalPoints);
                query.egoDestinationPoint = egoTarget;
                query.egoAlignPoint = egoBallPos;
                query.snapDistance = slowCatchRadius;
                query.additionalPoints = additionalPoints;
                query.velocityMode = msl::MovementQuery::Velocity::DEFAULT;
                mc = rm.moveToPoint(query);
            }

            // if we reached the point and are aligned, the behavior is successful
            if (egoTarget.length() < 120 && fabs(egoBallPos->rotateZ(M_PI).angleZ()) < (M_PI / 180) * alignTolerance)
            {
            	cout << "Position Executor: success" << endl;
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
    void PositionExecutor::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1438790362133) ENABLED START*/ //Add additional options here
        string receiverTaskName;
        if (getParameter("receiverTask", receiverTaskName))
        {
            receiverEp = this->getParentEntryPoint(receiverTaskName);
        }

        if (receiverEp == nullptr)
        { // there is no entrypoint with the receiver task given by the behaviour parameters
            auto parent = this->runningPlan->getParent().lock();
            if (parent != nullptr && ((Plan*)parent->getPlan())->getEntryPoints().size() == 2)
            { // there is only one other entry point than our own entry point, so it must be the receivers entry point.

                // which is my own entry point, so take the other one for the receiver
                auto activeEp = this->runningPlan->getActiveEntryPoint();
                auto eps = ((Plan*)parent->getPlan())->getEntryPoints();

                for (auto ep : eps)
                {
                    if (ep.first != activeEp->getId())
                    {
                        receiverEp = ep.second;
                        break;
                    }
                }
            }
            else
            {
                cerr << "PositionExecutor: Could not determine the receivers entry point!" << endl;
                throw std::runtime_error("PositionExecutor: Could not determine the receivers entry point!");
            }
        }
        else
        { // we found the entry point of the receiver, so everything is cool
        }

        // set some static member variables
        alloTargetVec = geometry::CNVecAllo(-500, 0);
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1438790362133) ENABLED START*/ //Add additional methods here
    void PositionExecutor::readConfigParameters()
    {
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        fastCatchRadius = (*sc)["Drive"]->get<double>("Drive.Fast.CatchRadius", NULL);
        slowCatchRadius = (*sc)["Drive"]->get<double>("Drive.Carefully.CatchRadius", NULL);
        alignTolerance = (*sc)["Drive"]->get<double>("Drive.Default.AlignTolerance", NULL);
        ballDistanceEx = (*sc)["Drive"]->get<double>("Drive.KickOff.BallDistEx", NULL);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
