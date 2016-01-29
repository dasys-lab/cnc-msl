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
        translation = (*this->sc)["Drive"]->get<double>("Duel.Velocity", NULL);
        fieldLength = (*this->sc)["Globals"]->get<double>("FootballField.FieldLength", NULL);
        fieldWidth = (*this->sc)["Globals"]->get<double>("FootballField.FieldWidth", NULL);
        direction = 0;
        itCounter = 0;
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
        shared_ptr < geometry::CNPosition > me = wm->rawSensorData.getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > ownPoint = make_shared < geometry::CNPoint2D > (me->x, me->y);
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();
        msl_actuator_msgs::MotionControl mc;

        if (me == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        shared_ptr < geometry::CNPoint2D > alloBallPos = egoBallPos->alloToEgo(*me);

        msl_actuator_msgs::BallHandleCmd bhc;
        bhc.leftMotor = -wheelSpeed;
        bhc.rightMotor = -wheelSpeed;
        send(bhc);

        //quickly drive toward the ball for a few seconds
        //TODO improve this

        if (itCounter++ < 8)
        {

            mc.motion.angle = egoBallPos->angleTo();
            mc.motion.rotation = 0;
            mc.motion.translation = 2 * translation;
            send(mc);
            return;
        }

        shared_ptr < geometry::CNPoint2D > ownGoalPos = make_shared < geometry::CNPoint2D > (-fieldLength / 2, 0.0);

        if (direction == 0)
        {
            if (ownGoalPos != nullptr && me->distanceTo(ownGoalPos) < fieldLength / 3)
            {
                //own goal is close, get the ball away

                if (checkSide(egoBallPos, ownGoalPos))
                {
                    direction = -1;
                }
                else
                {
                    direction = 1;
                }
            }
            else
            {

                //own goal is far away, look for nearby friends
                shared_ptr < geometry::CNPoint2D > friendly = nullptr;

                for (auto pos = wm->robots.getPositionsOfTeamMates()->begin();
                        pos != wm->robots.getPositionsOfTeamMates()->end(); pos++)
                {

                    shared_ptr < geometry::CNPoint2D > friendlyPos = make_shared < geometry::CNPoint2D
                            > (pos->get()->second->x, pos->get()->second->y);

                    if (friendlyPos->distanceTo(ownPoint) < 2000)
                    {
                        if (friendly == nullptr || friendly->distanceTo(ownPoint) < friendlyPos->distanceTo(ownPoint))
                        {
                            friendly = friendlyPos;
                        }
                    }

                    if (friendly != nullptr)
                    {
                        //found one, try to get the ball to him
                        if (checkSide(egoBallPos, friendly))
                        {
                            direction = 1;
                        }
                        else
                        {
                            direction = -1;
                        }
                    }

                    else if (me != nullptr)
                    {

                        bool posFree = true;
                        bool negFree = true;
                        //found none, looking for free space

                        //TODO allo oder ego? :(

                        //TODO anpassen sobald wir eine liste von gegnern im WM haben
                        for (auto it = wm->robots.getObstaclePoints(0)->begin();
                                it != wm->robots.getObstaclePoints(0)->end(); it++)
                        {

                            shared_ptr < geometry::CNPoint2D > obs = make_shared < geometry::CNPoint2D
                                    > (it->get()->x, it->get()->y);

                            if (obs->distanceTo(alloBallPos) < 2000 && fabs(obs->angleTo()) < M_PI / 3 * 2)
                            {
                                if (checkSide(alloBallPos, obs))
                                {
                                    posFree = false;
                                }
                                else
                                {
                                    negFree = false;
                                }
                            }

                        }
                        if (posFree)
                        {
                            direction = 1;
                        }
                        else if (negFree)
                        {
                            direction = -1;
                        }
                        else
                        {
                            //all occupied
                            if (me == nullptr)
                            {
                                //no idea
                                direction = 1;
                            }
                            else
                            {
                                shared_ptr < geometry::CNPoint2D > ballOrth1 = make_shared < geometry::CNPoint2D
                                        > (egoBallPos->y, -egoBallPos->x);
                                shared_ptr < geometry::CNPoint2D > ballOrth2 = make_shared < geometry::CNPoint2D
                                        > (-egoBallPos->y, egoBallPos->x);
                                ballOrth1 = ballOrth1->egoToAllo(*me);
                                ballOrth2 = ballOrth1->egoToAllo(*me);
                                double distance = distanceToFieldBorder(ownPoint, ballOrth1->angleTo());
                                if (distanceToFieldBorder(ownPoint, ballOrth2->angleTo()) < distance)
                                {
                                    direction = 1;
                                }
                                else
                                {
                                    direction = -1;
                                }

                            }
                        }
                    }
                }

            }
        }

        mc.motion.angle = egoBallPos->angleTo() + direction * M_PI;
        mc.motion.rotation = direction * M_PI / 2.0;
        mc.motion.translation = translation;

        send(mc);

        /*PROTECTED REGION END*/
    }
    void Duel::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1450178699265) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1450178699265) ENABLED START*/ //Add additional methods here
    // returns true if pointToCheck is left of lineVector(look along the lineVector)
    // uses cross product of 2 vectors. 0: colinear, <0: point left of vec, >0: point right of vec
    bool Duel::checkSide(shared_ptr<geometry::CNPoint2D> lineVector, shared_ptr<geometry::CNPoint2D> pointToCheck)
    {
        double cross = pointToCheck->x * lineVector->y - pointToCheck->y * lineVector->x;
        return (cross < 0);
    }

    double Duel::distanceToFieldBorder(shared_ptr<geometry::CNPoint2D> point, double angle)
    {

        shared_ptr < geometry::CNPoint2D > fieldCorner1 = make_shared < geometry::CNPoint2D
                > (-fieldLength / 2, fieldWidth / 2);
        shared_ptr < geometry::CNPoint2D > fieldCorner2 = make_shared < geometry::CNPoint2D
                > (fieldLength / 2, -fieldWidth / 2);

        double distance = 100000;
        double d;

        double vx = cos(angle);
        double vy = sin(angle);

        //Vector starts at (pointX,pointY), following it along a distance d gets you to (pointX + d*vx, pointY + d*vy).

        //Hitting left border
        d = (fieldCorner1->x - point->x) / vx;
        if (d > 0)
        {
            distance = min(distance, d);
        }

        //Right border
        d = (fieldCorner1->x - point->x) / vx;
        if (d > 0)
        {
            distance = min(distance, d);
        }

        //Top border
        d = (fieldCorner2->y - point->y) / vy;
        if (d > 0)
        {
            distance = min(distance, d);
        }

        //Bottom border
        d = (fieldCorner2->y - point->y) / vy;
        if (d > 0)
        {
            distance = min(distance, d);
        }

        return distance;

    }
/*PROTECTED REGION END*/
} /* namespace alica */
