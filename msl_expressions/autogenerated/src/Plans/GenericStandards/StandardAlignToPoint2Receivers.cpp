using namespace std;
#include "Plans/GenericStandards/StandardAlignToPoint2Receivers.h"

/*PROTECTED REGION ID(inccpp1467228931063) ENABLED START*/ // Add additional includes here
#include <Ball.h>
#include <MSLFootballField.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <GeometryCalculator.h>
#include <Rules.h>
#include <Robots.h>
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
    StandardAlignToPoint2Receivers::StandardAlignToPoint2Receivers() :
            DomainBehaviour("StandardAlignToPoint2Receivers")
    {
        /*PROTECTED REGION ID(con1467228931063) ENABLED START*/ // Add additional options here
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
        this->canPassCounter = 1;
        this->m_Query = make_shared<msl::MovementQuery>();
        // will be set in intialize parameters ...
        this->ballRadius = msl::Rules::getInstance()->getBallRadius();
        this->minOppDist = 0;
        this->passCorridorWidth = 0;
        this->canPassThreshold = 0;
        /*PROTECTED REGION END*/
    }
    StandardAlignToPoint2Receivers::~StandardAlignToPoint2Receivers()
    {
        /*PROTECTED REGION ID(dcon1467228931063) ENABLED START*/ // Add additional options here
        /*PROTECTED REGION END*/
    }
    void StandardAlignToPoint2Receivers::run(void* msg)
    {
        /*PROTECTED REGION ID(run1467228931063) ENABLED START*/ // Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = this->wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = this->wm->ball->getEgoBallPosition();

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

        EntryPoint *ep = getParentEntryPoint(this->teamMateTaskName1);
        if (ep != nullptr)
        {
            // get the plan in which the behavior is running
            auto parent = this->runningPlan->getParent().lock();
            if (parent == nullptr)
            {
                cout << "SATP2Rec: parent null" << endl;
                return;
            }
            // get robot ids of robots in found entry point
            shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep);
            // exactly one robot is receiver
            if (ids->size() > 0 && ids->at(0) != -1)
            {
                // get receiver position by id
                auto pos = this->wm->robots->teammates.getTeamMatePosition(ids->at(0));
                if (pos != nullptr)
                {
                    this->recPos1 = pos->getPoint();
                }
                else
                {
                    this->recPos1 = nullptr;
                }
            }
        }

        EntryPoint *ep2 = getParentEntryPoint(this->teamMateTaskName2);
        if (ep2 != nullptr)
        {
            // get the plan in which the behavior is running
            auto parent = this->runningPlan->getParent().lock();
            if (parent == nullptr)
            {
                cout << "SATP2Rec: parent null" << endl;
                return;
            }
            // get robot ids of robots in found entry point
            shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep2);
            // exactly one robot is receiver
            if (ids->size() > 0 && ids->at(0) != -1)
            {
                // get receiver position by id
                auto pos = this->wm->robots->teammates.getTeamMatePosition(ids->at(0));
                if (pos != nullptr)
                {
                    this->recPos2 = pos->getPoint();
                }
                else
                {
                    this->recPos2 = nullptr;
                }
            }
        }
        if (this->recPos1 == nullptr && this->recPos2 == nullptr)
        {
            return;
        }

        shared_ptr < geometry::CNPoint2D > passPoint = nullptr;
        if (this->recPos1->y < 0.0)
        {
            passPoint = make_shared < geometry::CNPoint2D
                    > (this->recPos1->x, -this->wm->field->getFieldWidth() / 2 + 1000.0);
        }
        else
        {
            passPoint = make_shared < geometry::CNPoint2D
                    > (this->recPos1->x, this->wm->field->getFieldWidth() / 2 - 1000.0);
        }
        if (!this->wm->field->isInsidePenalty(passPoint, 0.0))
        {

            // min dist to opponent
            auto obs = this->wm->robots->opponents.getOpponentsAlloClustered();
            bool opponentTooClose = false;
            for (int i = 0; i < obs->size(); i++)
            {
                if (obs->at(i)->distanceTo(passPoint) < this->minOppDist)
                {
                    opponentTooClose = true;
                    break;
                }
            }
            if (this->canPass && opponentTooClose)
            {
                this->canPass = false;
            }

            // some calculation to check whether any obstacle is inside the pass vector triangle
            shared_ptr < geometry::CNPoint2D > ball2PassPoint = passPoint - alloBall;
            double passLength = ball2PassPoint->length();
            shared_ptr < geometry::CNPoint2D > ball2PassPointOrth = make_shared < geometry::CNPoint2D
                    > (-ball2PassPoint->y, ball2PassPoint->x)->normalize() * this->ratio * passLength;
            shared_ptr < geometry::CNPoint2D > left = passPoint + ball2PassPointOrth;
            shared_ptr < geometry::CNPoint2D > right = passPoint - ball2PassPointOrth;
            if (this->canPass && !geometry::outsideTriangle(alloBall, right, left, this->ballRadius, obs)
                    && !geometry::outsideCorridore(alloBall, passPoint, this->passCorridorWidth, obs))
            {
                this->canPass = false;
            }
        }

        // Hack coimbra 17
        if (this->canPass)
        {
            this->canPassCounter = max(-4, min(this->canPassCounter + 1, 5));
        }
        else
        {
            this->canPassCounter = max(-4, min(this->canPassCounter - 1, 5));
        }
        shared_ptr < geometry::CNPoint2D > receiverPos = nullptr;
        if (this->canPassCounter > this->canPassThreshold)
        {
            this->canPassThreshold = -2;
            cout << "SAAG2R: aiming to receiver" << endl;
            receiverPos = this->recPos1;
        }
        else
        {
            this->canPassThreshold = 2;
            cout << "SAAG2R: aiming to alternative receiver" << endl;
            receiverPos = this->recPos2;
        }

        // calculate target executerDistanceToBall away from the ball and on a line with the receiver
        egoTarget = (alloBall + ((alloBall - receiverPos)->normalize() * this->executerDistanceToBall))->alloToEgo(
                *ownPos);

        // ask the path planner how to get there
        this->m_Query->egoDestinationPoint = egoTarget;
        this->m_Query->egoAlignPoint = receiverPos->alloToEgo(*ownPos);
        this->m_Query->additionalPoints = additionalPoints;
        mc = this->robot->robotMovement->moveToPoint(this->m_Query);

        // if we reach the point and are aligned, the behavior is successful
        if (egoTarget->length() < this->positionDistanceTolerance
                && fabs(receiverPos->alloToEgo(*ownPos)->angleTo()) < this->alignAngleTolerance)
        {
            this->setSuccess(true);
        }

        send(mc);

        /*PROTECTED REGION END*/
    }
    void StandardAlignToPoint2Receivers::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1467228931063) ENABLED START*/ // Add additional options here
        this->minOppDist = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "minOppDist", NULL);
        this->passCorridorWidth = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "passCorridorWidth", NULL);
        this->canPassCounter = 1;
        this->canPassThreshold = 0;
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

/*PROTECTED REGION END*/			
		} /* namespace alica */
