using namespace std;
#include "Plans/Standards/Own/Test/Align4PassTest.h"

/*PROTECTED REGION ID(inccpp1513609382468) ENABLED START*/ // Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/robotmovement/MovementQuery.h>
#include <msl_robot/MSLRobot.h>
#include <engine/RunningPlan.h>
#include <engine/Assignment.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <Robots.h>
#include <engine/Assignment.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1513609382468) ENABLED START*/ // initialise static variables here
    /*PROTECTED REGION END*/
    Align4PassTest::Align4PassTest() :
            DomainBehaviour("Align4PassTest")
    {
        /*PROTECTED REGION ID(con1513609382468) ENABLED START*/ // Add additional options here
        this->taskName = "";
        this->isReceiver = false;
        this->alignAngleTolerance = (M_PI / 180)
                * (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint", "alignAngleTolerance", NULL);
        this->positionDistanceTolerance = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint",
                                                                                        "positionDistanceTolerance",
                                                                                        NULL);
        this->executerDistanceToBall = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint",
                                                                                     "executerDistanceToBall", NULL);
        this->receiverDistanceToBall = (*this->sc)["StandardSituation"]->get<double>("PassTest", "ReceiverDistance",
                                                                                     NULL);
        this->receiverBallMovedThreshold = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint",
                                                                                         "receiverBallMovedThreshold",
                                                                                         NULL);
        /*PROTECTED REGION END*/
    }
    Align4PassTest::~Align4PassTest()
    {
        /*PROTECTED REGION ID(dcon1513609382468) ENABLED START*/ // Add additional options here
        /*PROTECTED REGION END*/
    }
    void Align4PassTest::run(void* msg)
    {
        /*PROTECTED REGION ID(run1513609382468) ENABLED START*/ // Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();

        // return if necessary information is missing
        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        // Create allo ball
        shared_ptr < geometry::CNPoint2D > alloBall = egoBallPos->egoToAllo(*ownPos);

        // Create additional points for path planning
        auto additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        // add alloBall to path planning
        additionalPoints->push_back(alloBall);
        shared_ptr < geometry::CNPoint2D > egoTarget;
        msl_actuator_msgs::MotionControl mc;
        if (!isReceiver)
        { // robot is executor

            // get entry point of task name to locate robot with task name
            EntryPoint *ep = getParentEntryPoint(taskName);
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
            auto receiverPos = wm->robots->teammates.getTeamMatePosition(ids->at(0));
            if (receiverPos == nullptr)
            {
                return;
            }

            // calculate target executerDistanceToBall away from the ball and on a line with the receiver
            egoTarget = (alloBall + ((alloBall - receiverPos)->normalize() * this->executerDistanceToBall))->alloToEgo(
                    *ownPos);
            egoTarget->y = alloBall->y;

            // ask the path planner how to get there
            this->m_Query->egoDestinationPoint = egoTarget;
            this->m_Query->egoAlignPoint = receiverPos->getPoint()->alloToEgo(*ownPos);
            this->m_Query->additionalPoints = additionalPoints;
            mc = this->robot->robotMovement->moveToPoint(m_Query);
        }
        else
        { // robot is receiver

            if (oldBallPos == nullptr)
            {
                oldBallPos = alloBall;
            }
            if (alloReceiverTarget == nullptr || oldBallPos->distanceTo(alloBall) > this->receiverBallMovedThreshold)
            { // recalculate alloReceiverTarget if the ball moved more than "receiverBallMovedThreshold" mm

                oldBallPos = alloBall;

                // calculate a point that is "receiverDistanceToBall" away from ball towards field mid (0,0).
                alloReceiverTarget = (alloBall + (alloBall->normalize() * -this->receiverDistanceToBall));
                alloReceiverTarget->y = alloBall->y;
                alloReceiverTarget = alloBall
                        + (alloBall - alloReceiverTarget)->normalize() * -this->receiverDistanceToBall;
            }

            // ask the path planner how to get there
            egoTarget = alloReceiverTarget->alloToEgo(*ownPos);
            this->m_Query->egoDestinationPoint = egoTarget;
            this->m_Query->egoAlignPoint = egoBallPos;
            this->m_Query->additionalPoints = additionalPoints;
            mc = this->robot->robotMovement->moveToPoint(m_Query);
        }

        // if we reach the point and are aligned, the behavior is successful
        if (egoTarget->length() < this->positionDistanceTolerance
                && fabs(egoBallPos->rotate(M_PI)->angleTo()) < this->alignAngleTolerance)
        {
            this->setSuccess(true);
        }

        send(mc);
        /*PROTECTED REGION END*/
    }
    void Align4PassTest::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1513609382468) ENABLED START*/ // Add additional options here
        this->m_Query = make_shared<msl::MovementQuery>();
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
/*PROTECTED REGION ID(methods1513609382468) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */