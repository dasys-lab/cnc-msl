using namespace std;
#include "Plans/GenericStandards/StandardAlignAndGrab2Receivers.h"

/*PROTECTED REGION ID(inccpp1462368682104) ENABLED START*/ // Add additional includes here
#include <Ball.h>
#include <Game.h>
#include <GeometryCalculator.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <Robots.h>
#include <engine/Assignment.h>
#include <engine/RunningPlan.h>
#include <engine/model/EntryPoint.h>
#include <engine/model/Plan.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/kicker/Kicker.h>
#include <msl_robot/robotmovement/MovementQuery.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <obstaclehandler/Obstacles.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1462368682104) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
StandardAlignAndGrab2Receivers::StandardAlignAndGrab2Receivers()
    : DomainBehaviour("StandardAlignAndGrab2Receivers")
{
    /*PROTECTED REGION ID(con1462368682104) ENABLED START*/ // Add additional options here
    this->ratio = 0.0;
    this->ballRadius = 0.0;
    this->minOppDist = 0.0;
    this->minCloserOffset = 0.0;
    this->passCorridorWidth = 0.0;
    this->maxTurnAngle = 0.0;
    this->longPassPossible = true;
    this->startTime = 0;
    this->tol = 0.0;
    this->minTol = 0.0;
    this->oldAngleErr = 0.0;
    this->angleIntErr = 0.0;
    this->trans = 0.0;
    this->haveBallCounter = 0;
    this->canPassCounter = 1;
    this->canPassThreshold = 1;
    this->query = make_shared<msl::MovementQuery>();
    /*PROTECTED REGION END*/
}
StandardAlignAndGrab2Receivers::~StandardAlignAndGrab2Receivers()
{
    /*PROTECTED REGION ID(dcon1462368682104) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void StandardAlignAndGrab2Receivers::run(void *msg)
{
    /*PROTECTED REGION ID(run1462368682104) ENABLED START*/ // Add additional options here
    shared_ptr<geometry::CNPosition> ownPos = this->wm->rawSensorData->getOwnPositionVision();
    shared_ptr<geometry::CNPoint2D> egoBallPos = this->wm->ball->getEgoBallPosition();
    // return if necessary information is missing
    if (ownPos == nullptr || egoBallPos == nullptr)
    {
        return;
    }

    this->longPassPossible = true;
    this->recPos = this->getTeammatePosFromTaskName(this->teamMateTaskName1);
    this->aRecPos = this->getTeammatePosFromTaskName(this->teamMateTaskName2);

    if (this->recPos == nullptr || this->aRecPos == nullptr)
    {
        return;
    }

    // make the passpoints closer to the receiver
    shared_ptr<geometry::CNPoint2D> passPoint = nullptr;
    if (this->recPos->y < 0.0)
    {
        passPoint = make_shared<geometry::CNPoint2D>(this->recPos->x, -this->wm->field->getFieldWidth() / 2.0 + 1000.0);
    }
    else
    {
        passPoint = make_shared<geometry::CNPoint2D>(this->recPos->x, this->wm->field->getFieldWidth() / 2.0 - 1000.0);
    }

    if (!this->wm->field->isInsidePenalty(passPoint, 0.0))
    {

        // min dist to opponent
        auto obs = wm->robots->opponents.getOpponentsAlloClustered();
        bool opponentTooClose = false;
        for (int i = 0; i < obs->size(); i++)
        {
            if (obs->at(i)->distanceTo(passPoint) < this->minOppDist)
            {
                opponentTooClose = true;
                break;
            }
        }
        if (opponentTooClose && this->longPassPossible)
        {
            this->longPassPossible = false;
        }

        shared_ptr<geometry::CNPoint2D> alloBall = egoBallPos->egoToAllo(*ownPos);
        // some calculation to check whether any obstacle is inside the pass vector triangle
        shared_ptr<geometry::CNPoint2D> ball2PassPoint = passPoint - alloBall;
        double passLength = ball2PassPoint->length();
        shared_ptr<geometry::CNPoint2D> ball2PassPointOrth =
            make_shared<geometry::CNPoint2D>(-ball2PassPoint->y, ball2PassPoint->x)->normalize() * this->ratio * passLength;
        shared_ptr<geometry::CNPoint2D> left = passPoint + ball2PassPointOrth;
        shared_ptr<geometry::CNPoint2D> right = passPoint - ball2PassPointOrth;
        if (this->longPassPossible && !geometry::outsideTriangle(alloBall, right, left, this->ballRadius, obs) &&
            !geometry::outsideCorridore(alloBall, passPoint, this->passCorridorWidth, obs))
        {
            this->longPassPossible = false;
        }
    }
    // Since coimbra 17
    if (this->longPassPossible)
    {
        this->canPassCounter = max(-40, min(this->canPassCounter + 1, 50));
    }
    else
    {
        this->canPassCounter = max(-40, min(this->canPassCounter - 1, 50));
    }

    shared_ptr<geometry::CNPoint2D> alloTarget = nullptr;
    if (this->canPassCounter > this->canPassThreshold)
    {
        this->canPassThreshold = -20;
//        cout << "SAAG2R: aiming to receiver" << endl;
        alloTarget = this->recPos;
    }
    else
    {
        this->canPassThreshold = 20;
//        cout << "SAAG2R: aiming to alternative receiver" << endl;
        alloTarget = this->aRecPos;
    }

    msl_actuator_msgs::MotionControl mc;
    if (egoBallPos->length() > 900.0)
    {
        // Drive close to the ball, until dist < 900
        this->query->egoDestinationPoint = egoBallPos;
        this->query->egoAlignPoint = egoBallPos;
        mc = this->robot->robotMovement->moveToPoint(query);

        if (mc.motion.angle != NAN)
        {
            send(mc);
        }
        else
        {
            cout << "SAAG2Rec: motion command is NaN!!" << endl;
        }
        return;
    }

    if (!this->wm->ball->haveBall())
    {
        this->haveBallCounter = 0;
    }

    if (egoBallPos->length() > 450.0)
    {
        // Drive closer to the ball, ###but don't rotate### DO ROTATE, we shouldn't be aligned soo poorly that the corner of the robot pushes away the ball at
        // this point
        this->query->egoDestinationPoint = egoBallPos;
        this->query->egoAlignPoint = egoBallPos;
        mc = this->robot->robotMovement->moveToPoint(query);

        // TODO this used to be uncommented.
        //        mc.motion.rotation = 0;
        mc.motion.translation = min(600.0, egoBallPos->length() / 1.66);
        if (mc.motion.angle != NAN)
        {
            sendAndUpdatePT(mc);
        }
        else
        {
            cout << "SAAG2Rec: motion command is NaN!!" << endl;
        }

        return;
    }

    double rot = this->trans / egoBallPos->length();

    shared_ptr<geometry::CNPoint2D> egoMatePos = alloTarget->alloToEgo(*ownPos);

    shared_ptr<geometry::CNPoint2D> direction = nullptr;

    double dangle = geometry::deltaAngle(this->robot->kicker->kickerAngle, egoMatePos->angleTo());

    double cross = egoMatePos->x * egoBallPos->y - egoMatePos->y * egoBallPos->x;
    double fac = cross > 0 ? 1 : -1;
    if (fabs(dangle) < 12.0 * M_PI / 180.0)
    {
        direction = egoBallPos->rotate(fac * M_PI / 2.0)->normalize() * this->trans * 0.66;
    }
    else
    {
        direction = egoBallPos->rotate(fac * M_PI / 2.0)->normalize() * this->trans;
    }

    double balldangle = geometry::deltaAngle(this->robot->kicker->kickerAngle, egoBallPos->angleTo());
    if (egoBallPos->length() > 350 && fabs(dangle) > 35.0 * M_PI / 180.0)
    {
        cout << "saag2rec: egoBallPos->length() > 350 && fabs(dangle) > 35.0 * M_PI / 180.0)" << endl;
        mc.motion.angle = direction->angleTo();
        mc.motion.translation = direction->length() * 1.6;
        mc.motion.rotation = -fac * rot * 1.6;
        sendAndUpdatePT(mc);
        return;
    }

    if (!this->wm->ball->haveBall())
    {
        if (fabs(balldangle) > 20.0 * M_PI / 180.0)
        {
//            cout << "saag2rec: !haveball && fabs(balldangle) > 20.0 * M_PI / 180.0)" << endl;
            mc.motion.rotation = (balldangle > 0 ? 1 : -1) * 0.8;
            mc.motion.angle = M_PI;
            mc.motion.translation = 100;
            sendAndUpdatePT(mc);
            return;
        }
        else
        {
//            cout << "not have ball else if" << endl;
            mc.motion.rotation = balldangle * 0.5;
            mc.motion.angle = egoBallPos->angleTo();
            mc.motion.translation = egoBallPos->length() * 1.5;
            sendAndUpdatePT(mc);
            return;
        }
    }

    this->angleIntErr += dangle;
    mc.motion.angle = direction->angleTo();
    mc.motion.translation = direction->length();
    mc.motion.rotation = -fac * rot * (2 * fabs(0.8 * dangle + 0.1 * this->angleIntErr + 2 * (dangle - this->oldAngleErr)));
    this->oldAngleErr = dangle;
    if (this->wm->ball->haveBall())
    {
        this->haveBallCounter++;
        double runningTimeMS = (double)((wm->getTime() - this->startTime) / 1000000ul);

        if (runningTimeMS > 6000.0 ||
            (haveBallCounter > 3 && ((runningTimeMS <= 3000.0 && fabs(dangle) < this->minTol) ||
                                     fabs(dangle) < this->minTol + max(0.0, (this->tol - this->minTol) / (3000.0 / (runningTimeMS - 3000.0))))))
        {
            mc.motion.angle = M_PI;
            mc.motion.rotation = 0.0;
            mc.motion.translation = 100.0;
            this->setSuccess(true);
        }
    }
    //		cout << "SAAG2R: last mc ROT: \t" << mc.motion.rotation << endl;
    sendAndUpdatePT(mc);

    /*PROTECTED REGION END*/
}
void StandardAlignAndGrab2Receivers::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1462368682104) ENABLED START*/ // Add additional options here
    this->haveBallCounter = 0;
    this->angleIntErr = 0;
    this->oldAngleErr = 0;
    this->startTime = this->wm->game->getTimeSinceStart();
    this->minTol = (*this->sc)["Behaviour"]->get<double>("StandardAlign.MinAlignTolerance", NULL);
    this->tol = (*this->sc)["Behaviour"]->get<double>("StandardAlign.AlignTolerance", NULL);
    this->minCloserOffset = (*this->sc)["Behaviour"]->get<double>("Pass", "MinCloserOffset", NULL);
    this->ballRadius = (*this->sc)["Rules"]->get<double>("Rules.BallRadius", NULL);
    this->ratio = tan((*this->sc)["Behaviour"]->get<double>("ThrowIn", "freeOppAngle", NULL) / 2);
    this->passCorridorWidth = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "passCorridorWidth", NULL);
    this->maxTurnAngle = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "maxTurnAngle", NULL);
    this->minOppDist = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "minOppDist", NULL);
    this->trans = (*this->sc)["Behaviour"]->get<double>("StandardAlign.AlignSpeed", NULL);
    string tmp;
    bool success = true;
    this->longPassPossible = true;
    this->canPassCounter = 1;
    this->canPassThreshold = 1;
    try
    {
        success &= getParameter("TeamMateTaskName1", tmp);
        if (success)
        {
            teamMateTaskName1 = tmp;
        }

        success &= getParameter("TeamMateTaskName2", tmp);
        if (success)
        {
            teamMateTaskName2 = tmp;
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
/*PROTECTED REGION ID(methods1462368682104) ENABLED START*/ // Add additional methods here
shared_ptr<geometry::CNPoint2D> StandardAlignAndGrab2Receivers::getTeammatePosFromTaskName(string teamMateTaskName)
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
