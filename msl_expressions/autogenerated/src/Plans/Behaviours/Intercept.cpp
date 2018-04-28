using namespace std;
#include "Plans/Behaviours/Intercept.h"

/*PROTECTED REGION ID(inccpp1458757170147) ENABLED START*/ // Add additional includes here
#include <Ball.h>
#include <Game.h>
#include <RawSensorData.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/kicker/Kicker.h>
#include <msl_robot/robotmovement/MovementQuery.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <pathplanner/PathPlanner.h>
#include <pathplanner/PathPlannerQuery.h>
#include <pathplanner/PathProxy.h>
#include <WhiteBoard.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1458757170147) ENABLED START*/ // initialise static variables here
    /*PROTECTED REGION END*/
    Intercept::Intercept() :
            DomainBehaviour("Intercept")
    {
        /*PROTECTED REGION ID(con1458757170147) ENABLED START*/ // Add additional options here
        this->pp = msl::PathProxy::getInstance();

        this->lastDistErr = 0.0;
        this->distIntErr = 0.0;

        this->lastRotErr = 0.0;
        this->rotIntErr = 0.0;

        this->maxBallVelocity = (*this->sc)["Drive"]->get<double>("Drive.Intercept.MaxBallVelocity", NULL);
        this->catchRadius = (*this->sc)["Drive"]->get<double>("Drive.Carefully.CatchRadius", NULL);
        this->mapInFieldOffset = (*this->sc)["Drive"]->get<double>("Drive.Intercept.mapInFieldOffset", NULL);

        this->prot = (*sc)["Drive"]->get<double>("Drive.Intercept.RotationP", NULL);
        this->pirot = (*sc)["Drive"]->get<double>("Drive.Intercept.RotationI", NULL);
        this->pdrot = (*sc)["Drive"]->get<double>("Drive.Intercept.RotationD", NULL);

        this->pdist = (*sc)["Drive"]->get<double>("Drive.Intercept.DistanceP", NULL);
        this->pidist = (*sc)["Drive"]->get<double>("Drive.Intercept.DistanceI", NULL);
        this->pddist = (*sc)["Drive"]->get<double>("Drive.Intercept.DistanceD", NULL);

        this->minDistErr = (*sc)["Drive"]->get<double>("Drive.Intercept.minDistErr", NULL);

        this->maxVel = (*sc)["Drive"]->get<double>("Drive.InterceptMaxSpeed", NULL);

        this->predictionTimestep = (*sc)["Drive"]->get<double>("Drive.Intercept.predictionTimestep", NULL);
        this->predictionHorizon = (*sc)["Drive"]->get<int>("Drive.Intercept.predictionHorizon", NULL);

        this->query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    Intercept::~Intercept()
    {
        /*PROTECTED REGION ID(dcon1458757170147) ENABLED START*/ // Add additional options here
        /*PROTECTED REGION END*/
    }
    void Intercept::run(void* msg)
    {
        /*PROTECTED REGION ID(run1458757170147) ENABLED START*/ // Add additional options here
        // ACQUIRE NECESSARY DATA
        auto ownPos = this->wm->rawSensorData->getOwnPositionVision();
        msl_actuator_msgs::MotionControl mc;
        if (ownPos == nullptr)
        {
            mc = this->robot->robotMovement->driveRandomly(500);
            send(mc);
            return;
        }

        auto egoBallPos = this->wm->ball->getEgoBallPosition();
        auto od = this->wm->rawSensorData->getCorrectedOdometryInfo();
        if (!egoBallPos || !od)
        {
            return;
        }

        auto egoBallVel = this->wm->ball->getVisionBallVelocity();
        if (!egoBallVel)
        {
            egoBallVel = make_shared < geometry::CNVelocity2D > (0, 0);
        }
        else if (egoBallVel->length() > this->maxBallVelocity)
        {
            egoBallVel = egoBallVel->normalize() * this->maxBallVelocity;
        }

        // Ball is outside field, so drive to its position mapped into field
        // Emergency-stop
        auto alloBall = egoBallPos->egoToAllo(*ownPos);
        if (!this->wm->field->isInsideField(alloBall))
        {
            auto egoTarget = this->wm->field->mapInsideField(alloBall, this->mapInFieldOffset)->alloToEgo(*ownPos);

            this->query->egoDestinationPoint = egoTarget;
            this->query->egoAlignPoint = egoBallPos;
            //            auto additonalPopints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
            //            additonalPopints->push_back(alloBall);
            //            this->query->additionalPoints = additonalPopints;
            mc = this->robot->robotMovement->moveToPoint(this->query);
            if (egoTarget->length() < this->catchRadius)
            {
                mc.motion.translation = 0;
                sendAndUpdatePT(mc);
            }
            else
            {
                send(mc);
            }

            return;
        }

        shared_ptr < geometry::CNPoint2D > predBall = make_shared < geometry::CNPoint2D
                > (egoBallPos->x, egoBallPos->y);
        //		if (egoBallVel->length() > 4000.0)
        //		{
        shared_ptr < geometry::CNPosition > predPos = make_shared < geometry::CNPosition > (0.0, 0.0, 0.0);
        double rot = od->motion.rotation * this->predictionTimestep / 1000.0;

        for (int i = 1; i < predictionHorizon; i++)
        {
            predPos->x += cos(od->motion.angle + predPos->theta) * od->motion.translation * this->predictionTimestep
                    / 1000.0;
            predPos->y += sin(od->motion.angle + predPos->theta) * od->motion.translation * this->predictionTimestep
                    / 1000.0;
            predPos->theta += rot;

            predBall->x += egoBallVel->x * this->predictionTimestep / 1000.0;
            predBall->y += egoBallVel->y * this->predictionTimestep / 1000.0;

            if (predBall->distanceTo(predPos)
                    < this->wm->pathPlanner->getRobotRadius() + this->wm->ball->getBallDiameter() / 2) // robotRadius+ballRadius
            {
                break;
            }
        }

        // to avoid crashing into the surrounding
        if (!this->wm->field->isInsideField(predPos->egoToAllo(*ownPos)))
        {
            msl_actuator_msgs::MotionControl mc;
            mc.motion.angle = 0;
            mc.motion.rotation = 0;
            mc.motion.translation = 0;
            sendAndUpdatePT(mc);
            return;
        }
        //		}
        // PID controller for minimizing the distance between ball and me
        double distErr = max(predBall->length(), this->minDistErr);
        double controlDist = distErr * this->pdist + this->distIntErr * this->pidist
                + (distErr - this->lastDistErr) * this->pddist;

        this->distIntErr += distErr - 1000.0; // reduce I part of the controller, when you are closer than 1 m to the ball
        this->distIntErr = max(0.0, min(800.0, this->distIntErr)); // never drive away from the ball, cause of the I part
        this->lastDistErr = distErr;

        bool ownPass = false;
        shared_ptr<msl_helper_msgs::PassMsg> passMsg = wm->whiteBoard->getPassMsg();
        if(passMsg!=nullptr)
        {
        	if (passMsg->receiverID == wm->getOwnId())
        	{
        		ownPass = true;
        	}
        }
        shared_ptr < geometry::CNPoint2D > egoVelocity;
        auto currentGameState = this->wm->game->getGameState();
        if (currentGameState == msl::GameState::OppBallPossession)
        {
            egoVelocity = make_shared < geometry::CNPoint2D > (0, 0);
        }
        //Hack torres vedras 18
        //if ball rolls towards me
        else if (ownPass && egoBallVel->x > 300 && egoBallPos->x < -200)
        {
            egoVelocity = make_shared < geometry::CNPoint2D > (0, 0);
            egoVelocity->y = egoBallVel->y;
        }
        else
        {
            egoVelocity = egoBallVel->getPoint();
        }
        //		cout << "Intercept: egoVelocity: " << egoVelocity->toString() << endl;
        egoVelocity->x += controlDist * cos(predBall->angleTo());
        egoVelocity->y += controlDist * sin(predBall->angleTo());
        //		cout << "Intercept: egoVelocity: " << egoVelocity->toString() << endl;

        auto pathPlanningPoint = egoVelocity->normalize() * min(egoVelocity->length(), predBall->length());
        auto alloDest = pathPlanningPoint->egoToAllo(*ownPos);
        if (this->wm->field->isInsideField(alloBall, -this->mapInFieldOffset)
                && !this->wm->field->isInsideField(alloDest))
        {
            // pathPlanningPoint = wm->field->mapInsideField((alloDest, alloBall - ownPos))->alloToEgo(*ownPos);
            pathPlanningPoint = this->wm->field->mapInsideField(alloDest, alloBall - alloDest)->alloToEgo(*ownPos);
        }

        shared_ptr < msl::PathEvaluator > eval = make_shared<msl::PathEvaluator>();
        shared_ptr < msl::PathPlannerQuery > query = make_shared<msl::PathPlannerQuery>();
        query->blockOppGoalArea = true;
        query->blockOwnGoalArea = true;
        auto pathPlanningResult = pp->getEgoDirection(pathPlanningPoint, eval, query);
        if (pathPlanningResult == nullptr)
        {
            mc.motion.angle = pathPlanningPoint->angleTo();
        }
        else
        {
            mc.motion.angle = pathPlanningResult->angleTo();
        }

        mc.motion.translation = min(this->maxVel, max(pathPlanningResult->length(), egoVelocity->length()));

        // PID controller for minimizing the kicker angle to ball
        double angleGoal = msl::Kicker::kickerAngle;
        double rotErr = geometry::deltaAngle(angleGoal, egoBallPos->angleTo());
        double controlRot = rotErr * this->prot + this->rotIntErr * this->pirot
                + (rotErr - this->lastRotErr) * this->pdrot;
        controlRot = max(-4 * M_PI, min(4 * M_PI, controlRot));

        this->rotIntErr += rotErr;
        this->rotIntErr = max(-2 * M_PI, min(2 * M_PI, this->rotIntErr));
        this->lastRotErr = rotErr;

        // this is nice stuff but only when we are not approaching the opponent
        if (egoBallPos->length() < 700
                && (currentGameState == msl::GameState::OwnBallPossession
                        || currentGameState == msl::GameState::NobodyInBallPossession))
        {
            controlRot *= 2.3;
            // we probably translate too fast and cannot rotate anymore: So translate slower
            if (fabs(rotErr) > M_PI / 6)
            {
                mc.motion.translation *= min((fabs(rotErr) - M_PI / 6) / (M_PI * 5.0 / 6.0), egoBallVel->length());
            }
        }
        mc.motion.rotation = controlRot;

        // Special handling for things around critical areas
        auto tmpMC = this->robot->robotMovement->ruleActionForBallGetter();
        if (!std::isnan(tmpMC.motion.translation))
        {
            send(tmpMC);
        }
        else
        {
            sendAndUpdatePT(mc);
            //        cout << "Intercept: Normal: " << mc.msotion.translation << endl;
        }

        //        if (this->wm->ball->haveBallDribble(false))
        //        {
        //            this->setSuccess(true);
        //        }
        if (this->wm->ball->haveBall())
        {
            this->setSuccess(true);
        }
        /*PROTECTED REGION END*/
    }
    void Intercept::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1458757170147) ENABLED START*/ // Add additional options here
        string tmp = "";
        if (getParameter("StandardSituationReceiver", tmp))
        {
            if (tmp.find("true") != string::npos || tmp.find("True") != string::npos)
            {
                predictionHorizon = (*this->sc)["Drive"]->get<int>(
                        "Drive.Intercept.StandardSituation.predictionHorizon", NULL);
            }
        }

        // only update when not in StandardAttack-Loop
        auto gs = this->wm->game->getGameState();
        if (gs != msl::GameState::OppBallPossession)
        {
            this->lastDistErr = 0.0;
            this->distIntErr = 0.0;

            this->lastRotErr = 0.0;
            this->rotIntErr = 0.0;
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1458757170147) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
