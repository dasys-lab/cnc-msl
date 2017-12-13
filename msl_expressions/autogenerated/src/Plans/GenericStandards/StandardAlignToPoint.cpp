using namespace std;
#include "Plans/GenericStandards/StandardAlignToPoint.h"

/*PROTECTED REGION ID(inccpp1433949970592) ENABLED START*/ // Add additional includes here
#include "msl_robot/robotmovement/MovementQuery.h"
#include "msl_robot/robotmovement/RobotMovement.h"
#include <Ball.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <Robots.h>
#include <engine/Assignment.h>
#include <engine/Assignment.h>
#include <engine/RunningPlan.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1433949970592) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
StandardAlignToPoint::StandardAlignToPoint()
    : DomainBehaviour("StandardAlignToPoint")
{
    /*PROTECTED REGION ID(con1433949970592) ENABLED START*/ // Add additional options here
    this->taskName = "";
    this->isReceiver = false;
    this->alignAngleTolerance = (M_PI / 180) * (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint", "alignAngleTolerance", NULL);
    this->positionDistanceTolerance = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint", "positionDistanceTolerance", NULL);
    this->executerDistanceToBall = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint", "executerDistanceToBall", NULL);
    this->receiverDistanceToBall = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint", "receiverDistanceToBall", NULL);
    this->receiverBallMovedThreshold = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint", "receiverBallMovedThreshold", NULL);
    /*PROTECTED REGION END*/
}
StandardAlignToPoint::~StandardAlignToPoint()
{
    /*PROTECTED REGION ID(dcon1433949970592) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void StandardAlignToPoint::run(void *msg)
{
    /*PROTECTED REGION ID(run1433949970592) ENABLED START*/ // Add additional options here
    shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision();
    shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball->getEgoBallPosition();

    // return if necessary information is missing
    if (ownPos == nullptr || egoBallPos == nullptr)
    {
        return;
    }

    // Create allo ball
    shared_ptr<geometry::CNPoint2D> alloBall = egoBallPos->egoToAllo(*ownPos);

    // Create additional points for path planning
    auto additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    // add alloBall to path planning
    additionalPoints->push_back(alloBall);
    shared_ptr<geometry::CNPoint2D> egoTarget;
    MotionControl mc;
    RobotMovement rm;
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
        egoTarget = (alloBall + ((alloBall - receiverPos)->normalize() * this->executerDistanceToBall))->alloToEgo(*ownPos);

        // ask the path planner how to get there
        this->m_Query->egoDestinationPoint = egoTarget;
        this->m_Query->egoAlignPoint = receiverPos->getPoint()->alloToEgo(*ownPos);
        this->m_Query->additionalPoints = additionalPoints;
        mc = rm.moveToPoint(m_Query);
    }
    else
    {   // robot is receiver
        /**
         * This is the default positioning of the receiver, if don't have a more clever behaviour.
         * Therefore, this part of this behaviour should be deleted the moment
         * we don't need the default positioning anymore.
         * -- Greetings Stopfer :P
         */
        if (oldBallPos == nullptr)
        {
            oldBallPos = alloBall;
        }
        if (alloReceiverTarget == nullptr || oldBallPos->distanceTo(alloBall) > this->receiverBallMovedThreshold)
        { // recalculate alloReceiverTarget if the ball moved more than "receiverBallMovedThreshold" mm

            oldBallPos = alloBall;

            // calculate a point that is "receiverDistanceToBall" away from ball towards field mid (0,0).
            alloReceiverTarget = (alloBall + (alloBall->normalize() * -this->receiverDistanceToBall));
        }

        // ask the path planner how to get there
        egoTarget = alloReceiverTarget->alloToEgo(*ownPos);
        this->m_Query->egoDestinationPoint = egoTarget;
        this->m_Query->egoAlignPoint = egoBallPos;
        this->m_Query->additionalPoints = additionalPoints;
        mc = rm.moveToPoint(m_Query);
    }

    // if we reach the point and are aligned, the behavior is successful
    if (egoTarget->length() < this->positionDistanceTolerance && fabs(egoBallPos->rotate(M_PI)->angleTo()) < this->alignAngleTolerance)
    {
    	cout << " SUCCESS SATP " << endl;
        this->setSuccess(true);
    }

    if (!std::isnan(mc.motion.translation))
    {
        send(mc);
    }
    else
    {
        cout << "SATP: translation nan" << endl;
    }

    /*PROTECTED REGION END*/
}
void StandardAlignToPoint::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1433949970592) ENABLED START*/ // Add additional options here
    this->m_Query = make_shared<MovementQuery>();
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
/*PROTECTED REGION ID(methods1433949970592) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
