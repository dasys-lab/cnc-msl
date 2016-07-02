using namespace std;
#include "Plans/GenericStandards/StandardAlignToPoint2Receivers.h"

/*PROTECTED REGION ID(inccpp1467228931063) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "msl_robot/robotmovement/MovementQuery.h"
#include <engine/RunningPlan.h>
#include <engine/Assignment.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <Robots.h>
#include <engine/Assignment.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1467228931063) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    StandardAlignToPoint2Receivers::StandardAlignToPoint2Receivers() :
            DomainBehaviour("StandardAlignToPoint2Receivers")
    {
        /*PROTECTED REGION ID(con1467228931063) ENABLED START*/ //Add additional options here
        this->alignAngleTolerance = (M_PI / 180)
                * (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint", "alignAngleTolerance", NULL);
        this->positionDistanceTolerance = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint",
                                                                                        "positionDistanceTolerance",
                                                                                        NULL);
        this->executerDistanceToBall = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint",
                                                                                     "executerDistanceToBall", NULL);
        this->receiverDistanceToBall = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint",
                                                                                     "receiverDistanceToBall", NULL);
        this->receiverBallMovedThreshold = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint",
                                                                                         "receiverBallMovedThreshold",
                                                                                         NULL);
        this->ratio = tan((*this->sc)["Behaviour"]->get<double>("ThrowIn", "freeOppAngle", NULL) / 2);
        this->canPass = true;
        /*PROTECTED REGION END*/
    }
    StandardAlignToPoint2Receivers::~StandardAlignToPoint2Receivers()
    {
        /*PROTECTED REGION ID(dcon1467228931063) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void StandardAlignToPoint2Receivers::run(void* msg)
    {
        /*PROTECTED REGION ID(run1467228931063) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();

        // return if necessary information is missing
        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }
        this->canPass = true;
        // Create allo ball
        shared_ptr < geometry::CNPoint2D > alloBall = egoBallPos->egoToAllo(*ownPos);

        // Create additional points for path planning
        auto additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        // add alloBall to path planning
        additionalPoints->push_back(alloBall);
        shared_ptr < geometry::CNPoint2D > egoTarget;
        MotionControl mc;
        RobotMovement rm;

        EntryPoint* ep = getParentEntryPoint(teamMateTaskName1);
        if (ep != nullptr)
        {
            // get the plan in which the behavior is running
            auto parent = this->runningPlan->getParent().lock();
            if (parent == nullptr)
            {
                cout << "parent null" << endl;
                return;
            }
            // get robot ids of robots in found entry point
            shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep);
            // exactly one robot is receiver
            if (ids->size() > 0 && ids->at(0) != -1)
            {
                // get receiver position by id
                auto pos = wm->robots->teammates.getTeamMatePosition(ids->at(0));
                if (pos != nullptr)
                {
                    recPos1 = make_shared < geometry::CNPoint2D > (pos->x, pos->y);
                }
                else
                {
                    recPos1 = nullptr;
                }
            }
        }

        EntryPoint* ep2 = getParentEntryPoint(teamMateTaskName2);
        if (ep2 != nullptr)
        {
            // get the plan in which the behavior is running
            auto parent = this->runningPlan->getParent().lock();
            if (parent == nullptr)
            {
                cout << "parent null" << endl;
                return;
            }
            // get robot ids of robots in found entry point
            shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep2);
            // exactly one robot is receiver
            if (ids->size() > 0 && ids->at(0) != -1)
            {
                // get receiver position by id
                auto pos = wm->robots->teammates.getTeamMatePosition(ids->at(0));
                if (pos != nullptr)
                {
                    recPos2 = make_shared < geometry::CNPoint2D > (pos->x, pos->y);
                }
                else
                {
                    recPos2 = nullptr;
                }
            }
        }
        if (recPos1 == nullptr && recPos2 == nullptr)
        {
            return;
        }

        shared_ptr < geometry::CNPoint2D > passPoint = nullptr;
        if (recPos1->y < 0.0)
        {
            passPoint = make_shared < geometry::CNPoint2D > (recPos1->x, -wm->field->getFieldWidth() / 2 + 1000.0);
        }
        else
        {
            passPoint = make_shared < geometry::CNPoint2D > (recPos1->x, wm->field->getFieldWidth() / 2 - 1000.0);
        }
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
            if (canPass && opponentTooClose)
            {
                canPass = false;
            }
//            if (canPass && geometry::absDeltaAngle(
//                    ownPos->theta + M_PI,
//                    (passPoint - make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y))->angleTo()) > maxTurnAngle)
//            {
//                canPass = false;
//            }

            // some calculation to check whether any obstacle is inside the pass vector triangle
            shared_ptr < geometry::CNPoint2D > ball2PassPoint = passPoint - alloBall;
            double passLength = ball2PassPoint->length();
            shared_ptr < geometry::CNPoint2D > ball2PassPointOrth = make_shared < geometry::CNPoint2D
                    > (-ball2PassPoint->y, ball2PassPoint->x)->normalize() * ratio * passLength;
            shared_ptr < geometry::CNPoint2D > left = passPoint + ball2PassPointOrth;
            shared_ptr < geometry::CNPoint2D > right = passPoint - ball2PassPointOrth;
            if (canPass && !outsideTriangle(alloBall, right, left, ballRadius, obs)
                    && !outsideCorridore(alloBall, passPoint, this->passCorridorWidth, obs))
            {
                canPass = false;
            }

        }
        shared_ptr < geometry::CNPoint2D > receiverPos = nullptr;
        if (canPass)
        {
            cout << "SAAG2R: aiming to receiver" << endl;
            receiverPos = recPos1;
        }
        else
        {
            cout << "SAAG2R: aiming to alternative receiver" << endl;
            receiverPos = recPos2;
        }

        // calculate target executerDistanceToBall away from the ball and on a line with the receiver
        egoTarget = (alloBall + ((alloBall - receiverPos)->normalize() * this->executerDistanceToBall))->alloToEgo(
                *ownPos);

        // ask the path planner how to get there
        this->m_Query->egoDestinationPoint = egoTarget;
        this->m_Query->egoAlignPoint = receiverPos->alloToEgo(*ownPos);
        this->m_Query->additionalPoints = additionalPoints;
        mc = rm.moveToPoint(m_Query);

        // if we reach the point and are aligned, the behavior is successful
        if (egoTarget->length() < this->positionDistanceTolerance
                && fabs(egoBallPos->rotate(M_PI)->angleTo()) < this->alignAngleTolerance)
        {
            this->setSuccess(true);
        }

        send(mc);

        /*PROTECTED REGION END*/
    }
    void StandardAlignToPoint2Receivers::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1467228931063) ENABLED START*/ //Add additional options here
        this->minOppDist = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "minOppDist", NULL);
        this->passCorridorWidth = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "passCorridorWidth", NULL);
        this->maxTurnAngle = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "maxTurnAngle", NULL);
        this->ballRadius = (*this->sc)["Rules"]->get<double>("Rules.BallRadius", NULL);
        this->m_Query = make_shared<MovementQuery>();
        this->alloReceiverTarget.reset();
        this->oldBallPos.reset();

        string tmp;
        bool success = true;
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
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "SAAG2R: Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1467228931063) ENABLED START*/ //Add additional methods here
    bool StandardAlignToPoint2Receivers::outsideCorridore(shared_ptr<geometry::CNPoint2D> ball,
                                                          shared_ptr<geometry::CNPoint2D> passPoint,
                                                          double passCorridorWidth,
                                                          shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points)
    {
        for (int i = 0; i < points->size(); i++)
        {
            if (geometry::distancePointToLineSegment(points->at(i)->x, points->at(i)->y, ball, passPoint)
            < passCorridorWidth)
            {
                return false;
            }
        }
        return true;
    }

    bool StandardAlignToPoint2Receivers::outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball,
                                                                   shared_ptr<geometry::CNPoint2D> passPoint,
                                                                   double passCorridorWidth,
                                                                   shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points)
    {
        for (int i = 0; i < points->size(); i++)
        {
            if (geometry::distancePointToLineSegment(points->at(i)->x, points->at(i)->y, ball, passPoint)
            < passCorridorWidth && ball->distanceTo(points->at(i)) < ball->distanceTo(passPoint) - 100)
            {
                return false;
            }
        }
        return true;
    }

    bool StandardAlignToPoint2Receivers::outsideTriangle(shared_ptr<geometry::CNPoint2D> a,
                                                         shared_ptr<geometry::CNPoint2D> b,
                                                         shared_ptr<geometry::CNPoint2D> c, double tolerance,
                                                         shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points)
                                                     {
                                                         shared_ptr<geometry::CNPoint2D> a2b = b - a;
                                                         shared_ptr<geometry::CNPoint2D> b2c = c - b;
                                                         shared_ptr<geometry::CNPoint2D> c2a = a - c;
                                                         shared_ptr<geometry::CNPoint2D> a2p;
                                                         shared_ptr<geometry::CNPoint2D> b2p;
                                                         shared_ptr<geometry::CNPoint2D> c2p;
                                                         shared_ptr<geometry::CNPoint2D> p;
                                                         for (int i = 0; i < points->size(); i++)
                                                         {
                                                             p = points->at(i);
                                                             a2p = p - a;
                                                             b2p = p - b;
                                                             c2p = p - c;

                                                             if ((a2p->x * a2b->y - a2p->y * a2b->x) / a2p->normalize()->length()< tolerance
						&& (b2p->x * b2c->y - b2p->y * b2c->x) / b2p->normalize()->length() < tolerance
						&& (c2p->x * c2a->y - c2p->y * c2a->x) / c2p->normalize()->length() < tolerance)
				{
					return false;
				}

			}
			return true;
		}
/*PROTECTED REGION END*/			
		} /* namespace alica */
