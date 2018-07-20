using namespace std;
#include "Plans/Behaviours/Duel.h"

/*PROTECTED REGION ID(inccpp1450178699265) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <obstaclehandler/Obstacles.h>
#include <Robots.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1450178699265) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Duel::Duel() :
            DomainBehaviour("Duel")
    {
        /*PROTECTED REGION ID(con1450178699265) ENABLED START*/ //Add additional options here
        wheelSpeed = (*this->sc)["Actuation"]->get<double>("Dribble.DuelWheelSpeed", NULL);
        translation = (*this->sc)["Behaviour"]->get<double>("Duel.Velocity", NULL);
        fieldLength = wm->field->getFieldLength();
        fieldWidth = wm->field->getFieldWidth();
        robotRadius = (*this->sc)["Rules"]->get<double>("Rules.RobotRadius", NULL);
        radiusToCheckOpp = (*this->sc)["Behaviour"]->get<double>("Duel.RadiusToCheckOpp", NULL);
        ;
        radiusToCheckOwn = (*this->sc)["Behaviour"]->get<double>("Duel.RadiusToCheckOwn", NULL);
        ;
        duelMaxTime = (*this->sc)["Behaviour"]->get<unsigned long>("Duel.DuelMaxTime", NULL);
        ;
        freeTime = 0;
        query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    Duel::~Duel()
    {
        /*PROTECTED REGION ID(dcon1450178699265) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Duel::run(void* msg)
    {
        /*PROTECTED REGION ID(run1450178699265) ENABLED START*/ //Add additional options here
        // enter plan when !haveBall && enemy haveBall || haveBall && enemy close
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();
        shared_ptr < geometry::CNPoint2D > egoTarget = nullptr;
        shared_ptr < geometry::CNPoint2D > egoAlignPoint = make_shared < geometry::CNPoint2D
                > (fieldLength / 2, 0)->alloToEgo(*ownPos);
//		shared_ptr<geometry::CNPoint2D> oppGoal = make_shared<geometry::CNPoint2D>(fieldLength / 2, 0);
        shared_ptr < geometry::CNPoint2D > oppGoal = wm->field->posOppGoalMid();
        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;

        shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> teamMatePositions = make_shared<
                vector<shared_ptr<geometry::CNPoint2D>>>();
        shared_ptr < geometry::CNPoint2D > closestFriendly = nullptr;

        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        //Check for Success
        auto obs = wm->obstacles->getEgoVisionObstaclePoints();
        if (obs != nullptr)
        {
            double distance = numeric_limits<double>::max();
            double temp;
            for (int i = 0; i < obs->size(); i++)
            {
                temp = obs->at(i)->distanceTo(egoBallPos);
                if (temp < distance && obs->at(i)->x < -300 && temp < 550)
                {
                    distance = temp;
                }
            }
            if (distance > 800) // && isTimeOut(1500000000, rp->getStateStartTime(), rp);
            {
                if (freeTime == 0)
                {
                    freeTime = wm->getTime();
                }
                if (wm->getTime() - freeTime > 330000000ul)
                {
                    this->setSuccess(true);
                }
            }
            else
            {
                freeTime = 0;
            }
        }
        //end successcheck

        shared_ptr < geometry::CNPoint2D > ownPoint = make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y);

        // push enemy robot and try to take the ball
//        if (!wm->ball.haveBall()
//                || (wm->ball.haveBall() && wm->game.getGameState() == msl::GameState::OppBallPossession))
//        {
//            mc.motion.translation = translation;
//            mc.motion.rotation = egoBallPos->rotate(M_PI)->angleTo() * 1.8;
//            mc.motion.angle = egoBallPos->angleTo();
//
//            bhc.leftMotor = -wheelSpeed;
//            bhc.rightMotor = -wheelSpeed;
//
//            send(bhc);
//            send(mc);
//
//        }
        bhc.leftMotor = -wheelSpeed;
        bhc.rightMotor = -wheelSpeed;
        send(bhc);

        if (wm->getTime() - entryTime < 000000)
        {
            mc.motion.translation = translation;
            mc.motion.rotation = 0;
            mc.motion.angle = egoBallPos->angleTo();

            send(mc);
        }
        else
        {
            shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> mostRecentOpps =
                    wm->robots->opponents.getOpponentsAlloClustered();
            shared_ptr < geometry::CNPoint2D > closestOpponent = nullptr;

            if (mostRecentOpps != nullptr)
            {
                for (auto opp : *mostRecentOpps)
                {
                    if (opp)
                    {

                        //find closest opponent in 2m radius to determine direction to move towards
                        if ((closestOpponent == nullptr
                                || opp->distanceTo(ownPoint) < closestOpponent->distanceTo(ownPoint))
                                && opp->distanceTo(ownPoint) < radiusToCheckOpp)
                        {
                            closestOpponent = opp;
                        }

                        teamMatePositions = wm->robots->teammates.getTeammatesAlloClustered();

                        // try to find closest team member that isn't blocked for aligning
                        for (int i = 0; i < teamMatePositions->size(); i++)
                        {
                            auto pos = teamMatePositions->at(i);
                            if (pos)
                            {
                                shared_ptr < geometry::CNPoint2D > friendlyPos = make_shared < geometry::CNPoint2D
                                        > (pos->x, pos->y);

                                //determine points for corridor check
                                shared_ptr < geometry::CNPoint2D > friendlyOrth1 = make_shared < geometry::CNPoint2D
                                        > (friendlyPos->y, -friendlyPos->x);
                                shared_ptr < geometry::CNPoint2D > friendlyOrth2 = make_shared < geometry::CNPoint2D
                                        > (-friendlyPos->y, friendlyPos->x);

                                shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> trianglePoints = make_shared<
                                        vector<shared_ptr<geometry::CNPoint2D>>>();

                                trianglePoints->push_back(
                                        friendlyPos + friendlyOrth1->normalize() * (robotRadius + 75));
                                trianglePoints->push_back(
                                        friendlyPos + friendlyOrth2->normalize() * (robotRadius + 75));
                                trianglePoints->push_back(ownPoint);

                                //check if opponent stands between me and my teammate
                                if (geometry::isInsidePolygon(*trianglePoints, opp))
                                {
                                    friendlyBlocked = true;
                                }

                                // find closest team member in 2m radius
                                if (friendlyPos->distanceTo(ownPoint) < radiusToCheckOwn)
                                {
                                    if (closestFriendly == nullptr
                                            || closestFriendly->distanceTo(ownPoint)
                                                    < friendlyPos->distanceTo(ownPoint))
                                    {
                                        closestFriendly = friendlyPos;
                                    }
                                }
                            }

                        }
                    }

                }

            }

            if (closestOpponent != nullptr)
            {
                hadClosestOpp.at(itcounter++ % 10) = true;
                itcounter++;

                //TODO cooleren punkt berechnen?
                //move away from opponent
                egoTarget = (closestOpponent->alloToEgo(*ownPos)->normalize()->rotate(M_PI)) * 2500;
                //egoTarget = (closestOpponent->alloToEgo(*ownPos) + oppGoal->alloToEgo(*ownPos))->normalize() * 2000;

            }
            else
            {

                hadClosestOpp.at(itcounter % 10) = false;
                itcounter++;

                //TODO hysteresis of last closest opps because sensor info might be faulty??

                //attempt at weighted analysis , not sure if working
                int counter = 0;

                for (int i = 0; i < hadClosestOpp.size(); i++)
                {
                    if (hadClosestOpp.at(i))
                    {
                        counter += hadClosestOpp.size() - i;
                    }
                    else
                    {
                        counter -= hadClosestOpp.size() - i;
                    }
                }

                if (counter <= 0)
                {
                    //cout << "Duel: Success, far away from opponent" << endl;
                    //this->success = true;
                    return;
                }

            }
            if (closestFriendly != nullptr && !friendlyBlocked)
            {
                //align to non-blocked closest team member for future pass play
                egoAlignPoint = closestFriendly->alloToEgo(*ownPos);
            }
            else if (closestFriendly != nullptr && friendlyBlocked)
            {

                //can't align to team member so align to opp goal
                egoAlignPoint = oppGoal->alloToEgo(*ownPos);
            }
            else
            {
                //found no team member at all
                cout << "Duel: Found nobody" << endl;
                if (ownPos == nullptr)
                {
                    //no idea
                    cout << "Duel: no idea" << endl;
                    egoAlignPoint = oppGoal->alloToEgo(*ownPos);
                }
                else
                {
                    //try closest field border

                    //TODO testen
                    cout << "Duel: try closest field border" << endl;

                    shared_ptr < geometry::CNPoint2D > ballOrth1 = make_shared < geometry::CNPoint2D
                            > (egoBallPos->y, -egoBallPos->x);
                    shared_ptr < geometry::CNPoint2D > ballOrth2 = make_shared < geometry::CNPoint2D
                            > (-egoBallPos->y, egoBallPos->x);
                    ballOrth1 = ballOrth1->egoToAllo(*ownPos);
                    ballOrth2 = ballOrth2->egoToAllo(*ownPos);

                    double distance = wm->field->distanceToLine(ownPoint, ballOrth1->egoToAllo(*ownPos)->angleTo());
                    if (wm->field->distanceToLine(ownPoint, ballOrth2->egoToAllo(*ownPos)->angleTo()) < distance)
                    {
                        //top line
                        egoAlignPoint = make_shared < geometry::CNPoint2D
                                > (ownPoint->x, -fieldWidth / 2)->alloToEgo(*ownPos);
                    }
                    else
                    {
                        //bottom line
                        egoAlignPoint = make_shared < geometry::CNPoint2D
                                > (ownPoint->x, fieldWidth / 2)->alloToEgo(*ownPos);
                    }
                }
            }

            if (!egoTarget)
            {
                egoTarget = (make_shared < geometry::CNPoint2D > (0, 0))->alloToEgo(*ownPos);
            }
            // replaced with new moveToPoint method
//            mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoTarget, 100);
            //mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoAlignPoint, 100);
            query->egoDestinationPoint = egoTarget;
            query->egoAlignPoint = egoTarget;
            query->snapDistance = 100;

            mc = this->robot->robotMovement->moveToPoint(query);

            if (!std::isnan(mc.motion.translation))
            {
                send(mc);
            }
            else
            {
                cout << "Motion command is NaN!" << endl;
            }
        }

        // too much time has passed, we don't want to stay in duel for too long (rules says sth like 10s until ball dropped)
        if (wm->getTime() - entryTime > duelMaxTime)
        {
            cout << "Duel: time over " << endl;
            this->setSuccess(true);
        }

        /*PROTECTED REGION END*/
    }
    void Duel::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1450178699265) ENABLED START*/ //Add additional options here
        freeTime = 0;
        direction = 0;
        friendlyBlocked = false;
        entryTime = wm->getTime();
        itcounter = 0;
        hadClosestOpp = vector<bool>(10, true);
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1450178699265) ENABLED START*/ //Add additional methods here
// returns true if pointToCheck is left of lineVector(look along the lineVector)
// uses cross product of 2 vectors. 0: colinear, <0: point left of vec, >0: point right of vec
    bool Duel::pointLeftOfVec(shared_ptr<geometry::CNPoint2D> lineVector, shared_ptr<geometry::CNPoint2D> pointToCheck)
    {
        double cross = pointToCheck->x * lineVector->y - pointToCheck->y * lineVector->x;
        return (cross < 0);
    }

/*PROTECTED REGION END*/
} /* namespace alica */
