using namespace std;
#include "Plans/GenericStandards/StandardAlignToPoint.h"

/*PROTECTED REGION ID(inccpp1433949970592) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include <engine/RunningPlan.h>
#include <engine/Assignment.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <Robots.h>
#include <engine/Assignment.h>
#include <MSLWorldModel.h>
using geometry::CNPointAllo;
using geometry::CNPointEgo;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1433949970592) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    StandardAlignToPoint::StandardAlignToPoint() :
            DomainBehaviour("StandardAlignToPoint")
    {
        /*PROTECTED REGION ID(con1433949970592) ENABLED START*/ //Add additional options here
        this->taskName = "";
        this->isReceiver = false;
        this->alignAngleTolerance = (M_PI / 180)
                * (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint", "alignAngleTolerance", NULL);
        this->positionDistanceTolerance = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint",
                                                                                        "positionDistanceTolerance",
                                                                                        NULL);
        this->executorDistanceToBall = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint",
                                                                                     "executerDistanceToBall", NULL);
        this->receiverDistanceToBall = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint",
                                                                                     "receiverDistanceToBall", NULL);
        this->receiverBallMovedThreshold = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint",
                                                                                         "receiverBallMovedThreshold",
                                                                                         NULL);
        this->alloReceiverTarget = nonstd::make_optional<CNPointAllo>();
        this->oldBallPos = nonstd::make_optional<CNPointAllo>();
        /*PROTECTED REGION END*/
    }
    StandardAlignToPoint::~StandardAlignToPoint()
    {
        /*PROTECTED REGION ID(dcon1433949970592) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void StandardAlignToPoint::run(void* msg)
    {
        /*PROTECTED REGION ID(run1433949970592) ENABLED START*/ //Add additional options here
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto egoBallPos = wm->ball->getPositionEgo();

        // return if necessary information is missing
        if (!ownPos || !egoBallPos)
        {
            return;
        }

        // Create allo ball
        auto alloBall = egoBallPos->toAllo(*ownPos);

        // Create additional points for path planning
        nonstd::optional<std::vector<geometry::CNPointAllo>> additionalPoints = nonstd::make_optional(
                vector<CNPointAllo>());
        // add alloBall to path planning
        additionalPoints->push_back(alloBall);
        CNPointEgo egoTarget;
        msl_actuator_msgs::MotionControl mc;
        msl::RobotMovement rm;
        if (!isReceiver)
        { // robot is executor

            // get entry point of task name to locate robot with task name
            EntryPoint* ep = getParentEntryPoint(taskName);
            if (ep == nullptr)
            {
                return;
            }

            // get the plan in which the behavior is running
            auto parent = this->runningPlan->getParent().lock();
            if (parent == nullptr)
            {
                return;
            }

            // get robot ids of robots in found entry point
            shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep);
            if (ids->empty() || ids->at(0) == -1)
            {
                return;
            }

            // get receiver position by id
            auto receiverPos = wm->robots->teammates.getTeammatePositionBuffer(ids->at(0)).getLastValidContent();
            if (!receiverPos)
            {
                return;
            }

            // calculate target executerDistanceToBall away from the ball and on a line with the receiver
            egoTarget =
                    (alloBall + ((alloBall - receiverPos->getPoint()).normalize() * this->executorDistanceToBall)).toEgo(
                            *ownPos);

            // ask the path planner how to get there
            this->m_Query.egoDestinationPoint = egoTarget;
            this->m_Query.egoAlignPoint = receiverPos->getPoint().toEgo(*ownPos);
            this->m_Query.additionalPoints = additionalPoints;
            mc = rm.moveToPoint(m_Query);
        }
        else
        { // robot is receiver
            /**
             * This is the default positioning of the receiver, if don't have a more clever behaviour.
             * Therefore, this part of this behaviour should be deleted the moment
             * we don't need the default positioning anymore.
             * -- Greetings Stopfer :P
             */
            if (!this->oldBallPos)
            {
                oldBallPos = alloBall;
            }
            if (!alloReceiverTarget || oldBallPos->distanceTo(alloBall) > this->receiverBallMovedThreshold)
            { // recalculate alloReceiverTarget if the ball moved more than "receiverBallMovedThreshold" mm

                oldBallPos = nonstd::make_optional<CNPointAllo>(alloBall);

                //calculate a point that is "receiverDistanceToBall" away from ball towards field mid (0,0).
                alloReceiverTarget = nonstd::make_optional<CNPointAllo>(
                        alloBall.x + (alloBall.normalize() * -this->receiverDistanceToBall).x,
                        alloBall.y + (alloBall.normalize() * -this->receiverDistanceToBall).y);
            }

            // ask the path planner how to get there
            egoTarget = alloReceiverTarget->toEgo(*ownPos);
            this->m_Query.egoDestinationPoint = egoTarget;
            this->m_Query.egoAlignPoint = egoBallPos;
            this->m_Query.additionalPoints = additionalPoints;
            mc = rm.moveToPoint(m_Query);
        }

        // if we reach the point and are aligned, the behavior is successful
        if (egoTarget.length() < this->positionDistanceTolerance
                && fabs(egoBallPos->rotateZ(M_PI).angleZ()) < this->alignAngleTolerance)
        {
            this->setSuccess(true);
        }

        send(mc);
        /*PROTECTED REGION END*/
    }
    void StandardAlignToPoint::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1433949970592) ENABLED START*/ //Add additional options here
        this->alloReceiverTarget.reset();
        this->oldBallPos.reset();

        string tmp = "";
        if (getParameter("TaskName", tmp))
        {
            taskName = tmp;
        }

        if (getParameter("Receiver", tmp))
        {
            if (tmp.find("true") != string::npos)
            {
                isReceiver = true;
            }
            else
            {
                isReceiver = false;
            }
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1433949970592) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
