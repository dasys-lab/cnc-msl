using namespace std;
#include "Plans/GenericStandards/StandardAlignToPoint2Receivers.h"

/*PROTECTED REGION ID(inccpp1467228931063) ENABLED START*/ // Add additional includes here
#include <Ball.h>
#include <GeometryCalculator.h>
#include <MSLFootballField.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <Robots.h>
#include <Rules.h>
#include <engine/Assignment.h>
#include <engine/Assignment.h>
#include <engine/RunningPlan.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/robotmovement/MovementQuery.h>
#include <msl_robot/robotmovement/RobotMovement.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1467228931063) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
StandardAlignToPoint2Receivers::StandardAlignToPoint2Receivers()
    : DomainBehaviour("StandardAlignToPoint2Receivers")
{
    /*PROTECTED REGION ID(con1467228931063) ENABLED START*/ // Add additional options here
    this->alignAngleTolerance = (M_PI / 180) * (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint", "alignAngleTolerance", NULL);
    this->positionDistanceTolerance = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint", "positionDistanceTolerance", NULL);
    this->executerDistanceToBall = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint", "executerDistanceToBall", NULL);
    this->receiverDistanceToBall = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint", "receiverDistanceToBall", NULL);
    this->receiverBallMovedThreshold = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint", "receiverBallMovedThreshold", NULL);
    this->ratio = tan((*this->sc)["Behaviour"]->get<double>("ThrowIn", "freeOppAngle", NULL) / 2);
    this->longPassPossible = true;
    this->longPassCounter = 1;
    this->m_Query = make_shared<msl::MovementQuery>();
    // will be set in intialize parameters ...
    this->ballRadius = msl::Rules::getInstance()->getBallRadius();
    this->minOppDist = 0;
    this->passCorridorWidth = 0;
    this->longPassThreshold = 0;
    /*PROTECTED REGION END*/
}
StandardAlignToPoint2Receivers::~StandardAlignToPoint2Receivers()
{
    /*PROTECTED REGION ID(dcon1467228931063) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void StandardAlignToPoint2Receivers::run(void *msg)
{
    /*PROTECTED REGION ID(run1467228931063) ENABLED START*/ // Add additional options here
    shared_ptr<geometry::CNPosition> ownPos = this->wm->rawSensorData->getOwnPositionVision();
    shared_ptr<geometry::CNPoint2D> egoBallPos = this->wm->ball->getEgoBallPosition();

    // return if necessary information is missing
    if (ownPos == nullptr || egoBallPos == nullptr)
    {
        return;
    }
    this->longPassPossible = true;
    // Create allo ball
    shared_ptr<geometry::CNPoint2D> alloBall = egoBallPos->egoToAllo(*ownPos);

    // Create additional points for path planning
    auto additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    // add alloBall to path planning
    additionalPoints->push_back(alloBall);
    shared_ptr<geometry::CNPoint2D> egoTarget;
    MotionControl mc;

    this->recPos = this->getTeammatePosFromTaskName(teamMateTaskName1);
    this->aRecPos = this->getTeammatePosFromTaskName(teamMateTaskName2);

    // LMA: changed from && to || because all plans with this beh require 3 robots as min cardinality. if one base crashes within the plan we get segfaults..
    if (this->recPos == nullptr || this->aRecPos == nullptr)
    {
        return;
    }

    if (!this->wm->field->isInsidePenalty(this->recPos, 0.0))
    {

        // min dist to opponent
        auto obs = this->wm->robots->opponents.getOpponentsAlloClustered();
        bool opponentTooClose = false;
        for (int i = 0; i < obs->size(); i++)
        {
            if (obs->at(i)->distanceTo(this->recPos) < this->minOppDist)
            {
                opponentTooClose = true;
                break;
            }
        }
        if (this->longPassPossible && opponentTooClose)
        {
            this->longPassPossible = false;
        }

        // some calculation to check whether any obstacle is inside the pass vector triangle
        shared_ptr<geometry::CNPoint2D> ball2PassPoint = this->recPos - alloBall;
        double passLength = ball2PassPoint->length();
        shared_ptr<geometry::CNPoint2D> ball2PassPointOrth =
            make_shared<geometry::CNPoint2D>(-ball2PassPoint->y, ball2PassPoint->x)->normalize() * this->ratio * passLength;
        shared_ptr<geometry::CNPoint2D> left = this->recPos + ball2PassPointOrth;
        shared_ptr<geometry::CNPoint2D> right = this->recPos - ball2PassPointOrth;
        if (this->longPassPossible && !geometry::outsideTriangle(alloBall, right, left, this->ballRadius, obs) &&
            !geometry::outsideCorridore(alloBall, this->recPos, this->passCorridorWidth, obs))
        {
            this->longPassPossible = false;
        }
    }

    // since coimbra 17
    if (this->longPassPossible)
    {
        this->longPassCounter = max(-40, min(this->longPassCounter + 1, 50));
    }
    else
    {
        this->longPassCounter = max(-40, min(this->longPassCounter - 1, 50));
    }
    shared_ptr<geometry::CNPoint2D> receiverPos = nullptr;
    if (this->longPassCounter > this->longPassThreshold)
    {
        this->longPassThreshold = -20;
        receiverPos = this->recPos;
    }
    else
    {
        this->longPassThreshold = 20;
        receiverPos = this->aRecPos;
    }

    // calculate target executerDistanceToBall away from the ball and on a line with the receiver
    egoTarget = (alloBall + ((alloBall - receiverPos)->normalize() * this->executerDistanceToBall))->alloToEgo(*ownPos);

    // ask the path planner how to get there
    this->m_Query->egoDestinationPoint = egoTarget;
    this->m_Query->egoAlignPoint = receiverPos->alloToEgo(*ownPos);
    this->m_Query->additionalPoints = additionalPoints;
    mc = this->robot->robotMovement->moveToPoint(this->m_Query);

    // if we reach the point and are aligned, the behavior is successful
    cout << egoTarget->length() << " < " << this->positionDistanceTolerance << endl;
    cout << fabs(receiverPos->alloToEgo(*ownPos)->rotate(M_PI)->angleTo()) << " < " << this->alignAngleTolerance << endl;

    if (egoTarget->length() < this->positionDistanceTolerance && fabs(receiverPos->alloToEgo(*ownPos)->rotate(M_PI)->angleTo()) < this->alignAngleTolerance)
    {
        this->setSuccess(true);
    }

    if (fabs(receiverPos->alloToEgo(*ownPos)->rotate(M_PI)->angleTo()) < 6 * this->alignAngleTolerance)
    {
        // weird hack to reach target alignment in simulator. relevant for real game?
        mc.motion.rotation *= 1.05;
        sendAndUpdatePT(mc);
    }

    send(mc);

    /*PROTECTED REGION END*/
}
void StandardAlignToPoint2Receivers::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1467228931063) ENABLED START*/ // Add additional options here
    this->minOppDist = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "minOppDist", NULL);
    this->passCorridorWidth = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "passCorridorWidth", NULL);
    this->longPassCounter = 1;
    this->longPassThreshold = 0;
    string tmp;
    bool success = true;
    try
    {
        success &= getParameter("TeamMateTaskName1", tmp);
        if (success)
        {
            this->teamMateTaskName1 = tmp;
        }

        success &= getParameter("TeamMateTaskName2", tmp);
        if (success)
        {
            this->teamMateTaskName2 = tmp;
        }
    }
    catch (exception &e)
    {
        cerr << "Could not cast the parameter properly" << endl;
    }
    if (!success)
    {
        cerr << "SAAG2R: Parameter does not exist" << endl;
    }
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1467228931063) ENABLED START*/ // Add additional methods here

shared_ptr<geometry::CNPoint2D> StandardAlignToPoint2Receivers::getTeammatePosFromTaskName(string teamMateTaskName)
{
    shared_ptr<geometry::CNPoint2D> recPos = nullptr;

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
