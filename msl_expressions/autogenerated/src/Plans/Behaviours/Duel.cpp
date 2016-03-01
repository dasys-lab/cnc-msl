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
        shared_ptr < geometry::CNPosition > me = wm->rawSensorData.getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > ownPoint = make_shared < geometry::CNPoint2D > (me->x, me->y);
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();
        shared_ptr < geometry::CNPoint2D > egoAlignPoint = nullptr;
        shared_ptr < geometry::CNPoint2D > egoTarget = make_shared < geometry::CNPoint2D > (0, 0);
        shared_ptr < geometry::CNPoint2D > friendly = nullptr;
        msl_actuator_msgs::MotionControl mc;

        if (me == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        shared_ptr < geometry::CNPoint2D > alloBallPos = egoBallPos->alloToEgo(*me);
        shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
                vector<shared_ptr<geometry::CNPoint2D>>>();

        additionalPoints->push_back(alloBallPos);

        msl_actuator_msgs::BallHandleCmd bhc;
        bhc.leftMotor = -wheelSpeed;
        bhc.rightMotor = -wheelSpeed;
        send(bhc);

        //quickly drive toward the ball for a few seconds
        //TODO improve this and set itCounter accordingly or check for haveball

//		if (itCounter++ < 200)
//		{
//
//			mc.motion.angle = egoBallPos->angleTo();
//			mc.motion.rotation = 0;
//			mc.motion.translation = 2 * translation;
//			send(mc);
//			return;
//		}

        shared_ptr < geometry::CNPoint2D > ownGoalPos = make_shared < geometry::CNPoint2D > (-fieldLength / 2, 0.0);
        shared_ptr < geometry::CNPoint2D > goalPosEgo = ownGoalPos->alloToEgo(*me);

        if (ownGoalPos != nullptr && ownPoint->distanceTo(ownGoalPos) < fieldLength / 3)
        {
            cout << "Duel: own goal is close" << endl;
            //own goal is close, get the ball away
            if (pointLeftOfVec(egoBallPos, goalPosEgo))
            {
                // goal is on the left side of a vector between me and the ball, so i turn right
                egoTarget = make_shared < geometry::CNPoint2D > (0, -fieldWidth / 2);
                egoAlignPoint = make_shared < geometry::CNPoint2D > (ownPoint->x, -fieldWidth / 2);

            }
            else
            {
                //goal on right side, turn left!
                egoAlignPoint = make_shared < geometry::CNPoint2D > (ownPoint->x, fieldWidth / 2);
                egoTarget = make_shared < geometry::CNPoint2D > (0, fieldWidth / 2);
            }
        }
        else
        {
            cout << "Duel: own Goal far away" << endl;
            //own goal is far away, look for nearby friends

            shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> teamMatePositions = make_shared<
                    vector<shared_ptr<geometry::CNPoint2D>>>();;
            int ownID = this->sc->getOwnRobotID();

            //mops 1, hairy 8, nase 9, savvy 10, myo 11
            //TODO dangerous on ID changes :(

            vector<int> teamMateIDs = vector<int> {8, 9, 10, 11};
            for (int id : teamMateIDs)
            {
                if (wm->robots.teammates.getTeamMatePosition(id, 0) != nullptr && id != ownID)
                {
                    shared_ptr < geometry::CNPoint2D > validPosition =
                            make_shared < geometry::CNPoint2D
                                    > (wm->robots.teammates.getTeamMatePosition(id, 0)->x, wm->robots.teammates.getTeamMatePosition(
                                            id, 0)->y);
                    teamMatePositions->push_back(validPosition);
                }
            }
            for (int i = 0; i < teamMatePositions->size(); i++)
            {

                auto pos = teamMatePositions->at(i);
                if (pos != nullptr)
                {
                    cout << "Duel: Positions of Teammates: X= " << pos->x << " Y= " << pos->y << endl;
                    cout << "Duel: Own Position: X= " << ownPoint->x << "Y= " << ownPoint->y << endl;

                    shared_ptr < geometry::CNPoint2D > friendlyPos = make_shared < geometry::CNPoint2D
                            > (pos->x, pos->y);

                    if (friendlyPos->distanceTo(ownPoint) < 2000)
                    {
                        if (friendly == nullptr || friendly->distanceTo(ownPoint) < friendlyPos->distanceTo(ownPoint))
                        {
                            friendly = friendlyPos;
                        }
                    }
                }
            }

            if (friendly != nullptr)
            {
                //found one, try to get the ball to him
                if (pointLeftOfVec(egoBallPos, friendly))
                {
                    cout << "Friendly is left of me and ball!" << endl;
//									egoAlignPoint = egoBallPos->rotate(M_PI / 2);
                    egoAlignPoint = friendly;
                }
                else if (!pointLeftOfVec(egoBallPos, friendly))
                {

                    cout << "Friendly is right of me and ball!" << endl;
//									egoAlignPoint = egoBallPos->rotate(-M_PI / 2);
                    egoAlignPoint = friendly;
                }
            }
            else if (me != nullptr)
            {
                bool leftFree = true;
                bool rightFree = true;
                //found none, looking for free space

                //TODO anpassen sobald wir eine liste von gegnern im WM haben

                //TODO schleife fixen

                for (auto it = wm->obstacles.getObstaclePoints(0)->begin(); it != wm->obstacles.getObstaclePoints(0)->end();
                        it++)
                {
                    //TODO friendly darf nicht obstacle sein

                    if (it->get() != nullptr)
                    {
                        cout << "Duel: it.get: " << it->get()->x << ", " << it->get()->y << endl;

                        shared_ptr < geometry::CNPoint2D > obs = make_shared < geometry::CNPoint2D
                                > (it->get()->x, it->get()->y);
                        shared_ptr < geometry::CNPoint2D > egoObs = obs->egoToAllo(*me);

                        if (obs->distanceTo(alloBallPos) < 2000 && fabs(obs->angleTo()) < M_PI / 3 * 2)
                        {
                            if (pointLeftOfVec(egoBallPos, obs))
                            {
                                leftFree = false;
                                cout << "Obstacle left of me and the ball!" << endl;
                            }
                            else
                            {
                                rightFree = false;
                                cout << "Obstacle right of me and the ball!" << endl;

                            }
                        }
                    }

                }
                if (leftFree)
                {
                    egoAlignPoint = egoBallPos->rotate(M_PI / 2);
                }
                else if (rightFree)
                {
                    egoAlignPoint = egoBallPos->rotate(-M_PI / 2);
                }
                else
                {
                    //all occupied
                    if (me == nullptr)
                    {
                        //no idea
                        cout << "Duel: no idea" << endl;
                        egoAlignPoint = egoBallPos->rotate(M_PI / 2);
                    }
                    else
                    {

                        cout << "Duel: try closest field border" << endl;

                        shared_ptr < geometry::CNPoint2D > ballOrth1 = make_shared < geometry::CNPoint2D
                                > (egoBallPos->y, -egoBallPos->x);
                        shared_ptr < geometry::CNPoint2D > ballOrth2 = make_shared < geometry::CNPoint2D
                                > (-egoBallPos->y, egoBallPos->x);
                        ballOrth1 = ballOrth1->egoToAllo(*me);
                        ballOrth2 = ballOrth1->egoToAllo(*me);
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
                }
            }

        }

        mc = msl::RobotMovement::moveToPointCarefully(egoTarget->alloToEgo(*me), egoAlignPoint, 0, additionalPoints);
        send(mc);

// exit plan when haveBall && enemy not close

        /*PROTECTED REGION END*/
    }
    void Duel::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1450178699265) ENABLED START*/ //Add additional options here
        itCounter = 0;
        direction = 0;
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
