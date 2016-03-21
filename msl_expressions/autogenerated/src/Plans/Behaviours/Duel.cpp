using namespace std;
#include "Plans/Behaviours/Duel.h"

/*PROTECTED REGION ID(inccpp1450178699265) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
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
        translation = (*this->sc)["Drive"]->get<double>("Drive.Duel.Velocity", NULL);
        fieldLength = (*this->sc)["Globals"]->get<double>("Globals.FootballField.FieldLength", NULL);
        fieldWidth = (*this->sc)["Globals"]->get<double>("Globals.FootballField.FieldWidth", NULL);
        robotRadius = (*this->sc)["Rules"]->get<double>("Rules.RobotRadius", NULL);
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
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();
        shared_ptr < geometry::CNPoint2D > egoTarget = nullptr;
        shared_ptr < geometry::CNPoint2D > egoAlignPoint = make_shared < geometry::CNPoint2D > (fieldLength / 2, 0);
        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;

        shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> teamMatePositions = make_shared<
                vector<shared_ptr<geometry::CNPoint2D>>>();
        shared_ptr < geometry::CNPoint2D > friendly = nullptr;

        double getAwayDistance = 2000;

        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            cout << "Duel: ownPos null or ballPos null" << endl;
            return;
        }
        shared_ptr < geometry::CNPoint2D > ownPoint = make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y);

        if (!wm->ball.haveBall() || (wm->ball.haveBall() && wm->ball.getOppBallPossession()))
        {
            mc.motion.translation = 2 * translation;
            mc.motion.rotation = egoBallPos->rotate(M_PI)->angleTo() * 1.8;
            mc.motion.angle = egoBallPos->angleTo();

            bhc.leftMotor = -wheelSpeed;
            bhc.rightMotor = -wheelSpeed;

            send(bhc);
            send(mc);

        }
        else
        {
//          shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> mostRecentObs = wm->obstacles.getAlloObstaclePoints();
            shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> mostRecentObs =
                    wm->robots.opponents.getOpponentsAlloClustered();
            shared_ptr < geometry::CNPoint2D > closestObstacle = nullptr;

            if (mostRecentObs != nullptr)
            {
                for (auto obstacle : *mostRecentObs)
                {
                    if (obstacle)
                    {

                        if ((closestObstacle == nullptr
                                || obstacle->distanceTo(ownPoint) < closestObstacle->distanceTo(ownPoint))
                                && obstacle->distanceTo(ownPoint) > 2000)
                        {
                            closestObstacle = obstacle;
                        }

                        teamMatePositions = wm->robots.teammates.getTeammatesAlloClustered();

                        //fyi
//						for (auto posit : *teamMatePositions)
//						{
//							if (posit)
//							{
//								cout << "Duel: teammate poses: " << posit->toString() << endl;
//							}
//						}
//						cout << "Duel : ownPos " << ownPos->toString() << endl;

                        for (int i = 0; i < teamMatePositions->size(); i++)
                        {
                            auto pos = teamMatePositions->at(i);
                            if (pos)
                            {
                                shared_ptr < geometry::CNPoint2D > friendlyPos = make_shared < geometry::CNPoint2D
                                        > (pos->x, pos->y);

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

                                if (geometry::isInsidePolygon(*trianglePoints, obstacle))
                                {
                                    friendlyBlocked = true;
                                }

//								if (friendlyPos->distanceTo(ownPoint) < 2000 && !friendlyBlocked)
//								cout << "Duel: distance to friendlyPos: " << friendlyPos->distanceTo(ownPoint) << endl;
                                if (friendlyPos->distanceTo(ownPoint) < 2000)
                                {
                                    if (friendly == nullptr
                                            || friendly->distanceTo(ownPoint) < friendlyPos->distanceTo(ownPoint))
                                    {
                                        friendly = friendlyPos;
                                    }
                                }
                            }

                        }
                    }

                }

            }

            if (closestObstacle != nullptr)
            {
                //TODO coolen punkt berechnen, funzt nicht wenn ein friendly im weg steht
                // TAKER oder funzt auch nicht wenn friendly null ist!!!
                cout << "Duel: closestObs " << closestObstacle->toString();
//                cout << "Duel: friendly " << friendly->toString(); <--- by taker
                egoTarget = (ownPoint + closestObstacle->rotate(M_PI)->normalize() * 300)->alloToEgo(*ownPos);
            }
            else
            {
                //TODO coolen punkt berechnen
                shared_ptr < geometry::CNPoint2D > center = make_shared < geometry::CNPoint2D > (0, 0);
                egoTarget = center->alloToEgo(*ownPos);

            }
            if (friendly != nullptr && !friendlyBlocked)
            {
                cout << "Duel: Found friendly that is not blocked!" << endl;
                cout << "Duel: Position of Friendly: " << friendly->toString() << endl;
                egoAlignPoint = friendly->alloToEgo(*ownPos);
            }
            else if (friendly != nullptr && friendlyBlocked)
            {
                cout << "Duel: Found friendly but unfortunately blocked!" << endl;
                cout << "Duel: Position of Friendly: " << friendly->toString() << endl;
                egoAlignPoint = egoTarget;
            }
            else
            {
                cout << "Duel: Found nobody" << endl;
                if (ownPos == nullptr)
                {
                    //no idea
                    cout << "Duel: no idea" << endl;
                    egoAlignPoint = egoBallPos->rotate(M_PI / 2);
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
                    ballOrth2 = ballOrth1->egoToAllo(*ownPos);
                    double distance = msl::MSLFootballField::distanceToLine(ownPoint, ballOrth1->angleTo());
                    if (msl::MSLFootballField::distanceToLine(ownPoint, ballOrth2->angleTo()) < distance)
                    {
                        egoAlignPoint = egoBallPos->rotate(M_PI / 2);
                    }
                    else
                    {
                        egoAlignPoint = egoBallPos->rotate(-M_PI / 2);
                    }

                }

                egoAlignPoint = egoTarget;
            }

//			if (closestObstacle != nullptr && closestObstacle->alloToEgo(*ownPos)->length() > getAwayDistance)
            if (egoTarget->length() < 50)
            {
                cout << "Duel: Distance to closest opp: " << closestObstacle->alloToEgo(*ownPos)->length() << endl;
                cout << "Duel success!" << endl;
                this->success = true;
                return;
            }

//            cout << "Duel: Moving away!" << endl;
            //TODO moveToPointFast??
            mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoAlignPoint, 25);
            send(mc);

        }

//		shared_ptr<geometry::CNPoint2D> ownGoalPos = make_shared<geometry::CNPoint2D>(-fieldLength / 2, 0.0);
//		shared_ptr<geometry::CNPoint2D> goalPosEgo = ownGoalPos->alloToEgo(*me);
//
//		if (ownGoalPos != nullptr && ownPoint->distanceTo(ownGoalPos) < fieldLength / 3)
//		{
//			cout << "Duel: own goal is close" << endl;
//			//own goal is close, get the ball away
//			if (pointLeftOfVec(egoBallPos, goalPosEgo))
//			{
//				cout << "Goal left!" << endl;
//				// goal is on the left side of a vector between me and the ball, so i turn right
//				egoAlignPoint = make_shared<geometry::CNPoint2D>(ownPoint->x, -fieldWidth / 2);
//
//			}
//			else
//			{
//
//				cout << "Goal right!" << endl;
//				//goal on right side, turn left!
//				egoAlignPoint = make_shared<geometry::CNPoint2D>(ownPoint->x, fieldWidth / 2);
//				egoTarget = make_shared<geometry::CNPoint2D>(0, fieldWidth / 2);
//			}
//		}

        /*PROTECTED REGION END*/
    }
    void Duel::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1450178699265) ENABLED START*/ //Add additional options here
        direction = 0;
        friendlyBlocked = false;
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
