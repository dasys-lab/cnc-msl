using namespace std;
#include "Plans/Standards/Own/ThrowIn/ThrowInPass.h"

/*PROTECTED REGION ID(inccpp1462363192018) ENABLED START*/ // Add additional includes here
#include "engine/Assignment.h"
#include "engine/RunningPlan.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "msl_robot/robotmovement/RobotMovement.h"
#include <Ball.h>
#include <Game.h>
#include <GeometryCalculator.h>
#include <MSLFootballField.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <Robots.h>
#include <Rules.h>
#include <msl_helper_msgs/PassMsg.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/kicker/Kicker.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1462363192018) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
ThrowInPass::ThrowInPass()
    : DomainBehaviour("ThrowInPass")
{
    /*PROTECTED REGION ID(con1462363192018) ENABLED START*/ // Add additional options here
    this->ratio = 0;
    this->closerFactor = 0;
    this->ballRadius = msl::Rules::getInstance()->getBallRadius();
    this->minOppDist = 0;
    this->passCorridorWidth = 0;
    this->maxTurnAngle = 0;
    this->longPassPossible = true;
    this->maxVel = 2000;
    this->pRot = 2.1;
    this->dRot = 0.0;
    this->lastRotError = 0;
    this->minRot = 0.1;
    this->maxRot = M_PI * 4;
    this->accel = 2000;
    this->arrivalTimeOffset = NAN;
    /*PROTECTED REGION END*/
}
ThrowInPass::~ThrowInPass()
{
    /*PROTECTED REGION ID(dcon1462363192018) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void ThrowInPass::run(void *msg)
{
    /*PROTECTED REGION ID(run1462363192018) ENABLED START*/ // Add additional options here
    shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision();
    shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball->getEgoBallPosition();
    // return if necessary information is missing
    if (ownPos == nullptr || egoBallPos == nullptr)
    {
        return;
    }

    if (this->sentPm && !this->wm->ball->haveBall())
    {
        this->setSuccess(true);
    }

    shared_ptr<geometry::CNPoint2D> alloBall = egoBallPos->egoToAllo(*ownPos);

    auto recInf = this->getTeammateIdAndPosFromTaskName(teamMateTaskName1);
    auto aRecInf = this->getTeammateIdAndPosFromTaskName(teamMateTaskName2);
    this->recId = recInf.first;
    this->aRecId = aRecInf.first;
    this->recPos = recInf.second;
    this->aRecPos = aRecInf.second;

    if (!recPos || !aRecPos)
    {
        cout << "ThrowInPass: Could not get Receiver Positions from Task Names" << endl;
        return;
    }
    // make the passpoints closer to the receiver
    shared_ptr<geometry::CNPoint2D> passPoint = nullptr;

    /* this didn't work in testing
     if (recPos->y < 0.0)
     {
     passPoint = make_shared<geometry::CNPoint2D>(recPos->x, -wm->field->getFieldWidth() / 2 + 1000.0);
     }
     else
     {
     passPoint = make_shared<geometry::CNPoint2D>(recPos->x, wm->field->getFieldWidth() / 2 - 1000.0);
     }
     */

    if (!wm->field->isInsidePenalty(passPoint, 0.0))
    {

        // min dist to opponent
        auto obs = wm->robots->opponents.getOpponentsAlloClustered();
        bool opponentTooClose = false;
        for (int i = 0; i < obs->size(); i++)
        {
            if (obs->at(i)->distanceTo(passPoint) < minOppDist)
            {
                opponentTooClose = true;
                break;
            }
        }
        if (longPassPossible && opponentTooClose)
        {
            longPassPossible = false;
        }
        // some calculation to check whether any opponent is inside the pass vector triangle
        shared_ptr<geometry::CNPoint2D> ball2PassPoint = passPoint - alloBall;
        shared_ptr<geometry::CNPoint2D> ball2PassPointOrth =
            make_shared<geometry::CNPoint2D>(-ball2PassPoint->y, ball2PassPoint->x)->normalize() * ratio * ball2PassPoint->length();
        shared_ptr<geometry::CNPoint2D> left = passPoint + ball2PassPointOrth;
        shared_ptr<geometry::CNPoint2D> right = passPoint - ball2PassPointOrth;
        if (longPassPossible && !geometry::outsideTriangle(alloBall, right, left, ballRadius, obs) &&
            !geometry::outsideCorridore(alloBall, passPoint, this->passCorridorWidth, obs))
        {
            longPassPossible = false;
        }

        // no opponent was in dangerous distance to our pass vector, now check our teammates with other parameters
        auto matePoses = wm->robots->teammates.getTeammatesAlloClustered();
        if (longPassPossible && matePoses != nullptr && !outsideCorridoreTeammates(alloBall, passPoint, this->ballRadius * 4, matePoses))
        {
            longPassPossible = false;
        }
    }

    int bestReceiverId = -1;
    shared_ptr<geometry::CNPoint2D> alloTarget = nullptr;
    if (longPassPossible)
    {
        alloTarget = recPos;
        bestReceiverId = recId;
    }
    else
    {
        alloTarget = aRecPos;
        bestReceiverId = aRecId;
    }

    shared_ptr<geometry::CNPoint2D> aimPoint = passPoint->alloToEgo(*ownPos);
    double aimAngle = aimPoint->angleTo();
    double ballAngle = egoBallPos->angleTo();
    double deltaAngle = geometry::deltaAngle(ballAngle, aimAngle);
    // coimbra timeout hack, Stopfer said push it.
    if (abs(deltaAngle) < M_PI / 36 || this->wm->getTime() - this->wm->game->getTimeSinceStart() > 5e9)
    { // +/-5 degree
        // Kick && PassMsg
        msl_helper_msgs::PassMsg pm;
        msl_msgs::Point2dInfo pinf;
        // Distance to aim point * direction of our kicker = actual pass point destination
        double dist = aimPoint->length();
        shared_ptr<geometry::CNPoint2D> dest = make_shared<geometry::CNPoint2D>(-dist, 0);
        dest = dest->egoToAllo(*ownPos);
        pinf.x = dest->x;
        pinf.y = dest->y;
        pm.destination = pinf;
        pinf = msl_msgs::Point2dInfo();
        pinf.x = ownPos->x;
        pinf.y = ownPos->y;
        pm.origin = pinf;
        pm.receiverID = bestReceiverId;
        msl_actuator_msgs::KickControl km;
        km.enabled = true;
        km.kicker = 1; //(ushort)KickHelper.KickerToUseIndex(egoBallPos->angleTo());

        shared_ptr<geometry::CNPoint2D> goalReceiverVec = dest - make_shared<geometry::CNPoint2D>(alloTarget->x, alloTarget->y);

        // wat?
        double v0 = 0;
        double distReceiver = goalReceiverVec->length();
        double estimatedTimeForReceiverToArrive = (sqrt(2 * accel * distReceiver + v0 * v0) - v0) / accel;
        // considering network delay and reaction time 1s?:
        estimatedTimeForReceiverToArrive += 1.0;
        pm.validFor = (unsigned long long)(estimatedTimeForReceiverToArrive * 1000000000.0 + 300000000.0); // this is sparta!
                                                                                                           // if (closerFactor < 0.01)
        //{
        km.power = (ushort) this->robot->kicker->getKickPowerPass(aimPoint->length());
        /*}
         else
         {
         km.power = (ushort) this->robot->kicker->getPassKickpower(dist, estimatedTimeForReceiverToArrive + arrivalTimeOffset);
         }
         */

        send(km);
        if (this->robot->kicker->lowShovelSelected)
        {
            send(pm);
            this->sentPm = true;
        }
    }

    auto dstscan = this->wm->rawSensorData->getDistanceScan();
    if (dstscan != nullptr && dstscan->size() != 0)
    {
        double distBeforeBall = msl::Kicker::minFree(egoBallPos->angleTo(), 200, dstscan);
        if (distBeforeBall < 250.0)
            this->setFailure(true);
    }

    msl_actuator_msgs::MotionControl mc;
    mc.motion.rotation = deltaAngle * pRot + (deltaAngle - lastRotError) * dRot;
    mc.motion.rotation = geometry::sgn(mc.motion.rotation) * min(this->maxRot, max(abs(mc.motion.rotation), this->minRot));
    lastRotError = deltaAngle;
    double transBallOrth = egoBallPos->length() * mc.motion.rotation; // may be negative!

    auto ballVel = this->wm->ball->getVisionBallVelocity();
    if (ballVel == nullptr)
    {
        ballVel = make_shared<geometry::CNVelocity2D>(0, 0);
    }

    double transBallTo = min(1000.0, ballVel->length());
    auto driveTo = egoBallPos->rotate(-M_PI / 2.0);
    driveTo = driveTo->normalize() * transBallOrth;
    driveTo = driveTo + egoBallPos->normalize() * transBallTo;
    mc.motion.angle = driveTo->angleTo();
    mc.motion.translation = min(driveTo->length(), maxVel);

    sendAndUpdatePT(mc);

    /*PROTECTED REGION END*/
}
void ThrowInPass::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1462363192018) ENABLED START*/ // Add additional options here
    this->sentPm = false;
    this->closerFactor = (*this->sc)["Behaviour"]->get<double>("Pass", "CloserFactor", NULL);
    this->ratio = tan((*this->sc)["Behaviour"]->get<double>("ThrowIn", "freeOppAngle", NULL) / 2);
    this->passCorridorWidth = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "passCorridorWidth", NULL);
    this->maxTurnAngle = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "maxTurnAngle", NULL);
    this->minOppDist = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "minOppDist", NULL);
    this->maxVel = (*this->sc)["Behaviour"]->get<double>("Behaviour", "MaxSpeed", NULL);
    this->pRot = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "RotationP", NULL);
    this->dRot = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "RotationD", NULL);
    this->minRot = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "MinRotation", NULL);
    this->maxRot = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "MaxRotation", NULL);
    this->accel = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "ReceiverRobotAcceleration", NULL);
    this->arrivalTimeOffset = (*this->sc)["Behaviour"]->get<double>("Pass", "ArrivalTimeOffset", NULL);
    this->longPassPossible = true;

    string tmp;
    bool success = true;
    try
    {
        success &= getParameter("TeamMateTaskName1", tmp);
        if (success)
        {
            teamMateTaskName1 = tmp;
        }
        tmp = "";
        success &= getParameter("TeamMateTaskName2", tmp);
        if (success)
        {
            teamMateTaskName2 = tmp;
        }
        cout << "TASKNAMES: " << teamMateTaskName1 << "," << teamMateTaskName2 << endl;
    }
    catch (exception &e)
    {
        cerr << "Could not cast the parameter properly" << endl;
    }
    if (!success)
    {
        cerr << "SA2P: Parameter does not exist" << endl;
    }

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1462363192018) ENABLED START*/ // Add additional methods here
pair<int, shared_ptr<geometry::CNPoint2D>> ThrowInPass::getTeammateIdAndPosFromTaskName(string teamMateTaskName)
{
    int recId = -1;
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
                auto pos = wm->robots->teammates.getTeamMatePosition(ids->at(0));
                if (pos != nullptr)
                {
                    recId = ids->at(0);
                    recPos = pos->getPoint();
                }
            }
        }
    }


    return make_pair(recId, recPos);
}



bool ThrowInPass::outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball, shared_ptr<geometry::CNPoint2D> passPoint, double passCorridorWidth,
                                            shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points)
{
    for (int i = 0; i < points->size(); i++)
    {
        if (geometry::distancePointToLineSegment(points->at(i)->x, points->at(i)->y, ball, passPoint) < passCorridorWidth &&
            ball->distanceTo(points->at(i)) < ball->distanceTo(passPoint) - 100)
        {
            return false;
        }
    }
    return true;
}

/*PROTECTED REGION END*/
} /* namespace alica */
