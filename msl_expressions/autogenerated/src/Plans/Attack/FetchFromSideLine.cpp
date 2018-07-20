using namespace std;
#include "Plans/Attack/FetchFromSideLine.h"

/*PROTECTED REGION ID(inccpp1450175655102) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1450175655102) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    FetchFromSideLine::FetchFromSideLine() :
            DomainBehaviour("FetchFromSideLine")
    {
        /*PROTECTED REGION ID(con1450175655102) ENABLED START*/ //Add additional options here
        threshold = 400;
        behindDistance = 300;
        maxVel = 3000;

        query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    FetchFromSideLine::~FetchFromSideLine()
    {
        /*PROTECTED REGION ID(dcon1450175655102) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void FetchFromSideLine::run(void* msg)
    {
        /*PROTECTED REGION ID(run1450175655102) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision(); //OwnPositionCorrected;
        if (ownPos == nullptr)
        {
            this->setFailure(true);
            return;
        }

        shared_ptr < geometry::CNPoint2D > ballPos = wm->ball->getEgoBallPosition();
        if (ballPos == nullptr)
        {
            this->setFailure(true);
            return;
        }
        auto bm = this->robot->robotMovement->ruleActionForBallGetter();

        if (!std::isnan(bm.motion.translation))
        {
            send(bm);
            return;
        }

        msl_actuator_msgs::MotionControl mc;
        shared_ptr < geometry::CNPoint2D > alloBall = ballPos->egoToAllo(*ownPos);
        shared_ptr < geometry::CNPoint2D > dest = make_shared<geometry::CNPoint2D>();
        if (nearSideLine (alloBall))
        {
            if (alloBall->y < 0)
            {
                if (ownPos->y < alloBall->y - behindDistance)
                {
                    this->setSuccess(true);
                }
                dest->x = alloBall->x;
                dest->y = alloBall->y - behindDistance;
            }
            else
            {
                if (ownPos->y > alloBall->y + behindDistance)
                {
                    this->setSuccess(true);
                }
                dest->x = alloBall->x;
                dest->y = alloBall->y + behindDistance;
            }
            shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
                    vector<shared_ptr<geometry::CNPoint2D>>>();
            additionalPoints->push_back(alloBall);

            // replaced with new moveToPoint method
//            bm = msl::RobotMovement::moveToPointCarefully(dest->alloToEgo(*ownPos), dest->alloToEgo(*ownPos), 0,
//                                                          additionalPoints);
            query->egoDestinationPoint = dest->alloToEgo(*ownPos);
            query->egoAlignPoint = dest->alloToEgo(*ownPos);
            query->additionalPoints = additionalPoints;
            bm = this->robot->robotMovement->moveToPoint(query);

            //DriveHelper.DriveToPointAndAlignCareBall(WorldHelper.Allo2Ego(dest, ownPos), ballPos, maxVel, WM);
        }
        if (nearXLine (alloBall))
        {
            if (alloBall->x < 0)
            {
                if (ownPos->x < alloBall->x - behindDistance)
                {
                    this->setSuccess(true);
                }
                dest->x = alloBall->x - behindDistance;
                dest->y = alloBall->y;
            }
            else
            {
                if (ownPos->x > alloBall->x + behindDistance)
                {
                    this->setSuccess(true);
                }
                dest->x = alloBall->x + behindDistance;
                dest->y = alloBall->y;
            }
            shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
                    vector<shared_ptr<geometry::CNPoint2D>>>();
            additionalPoints->push_back(alloBall);

            // replaced with new moveToPointMethod
//            bm = msl::RobotMovement::moveToPointCarefully(dest->alloToEgo(*ownPos), dest->alloToEgo(*ownPos), 0,
//                                                          additionalPoints);
            query->egoDestinationPoint = dest->alloToEgo(*ownPos);
            query->egoAlignPoint = dest->alloToEgo(*ownPos);
            query->additionalPoints = additionalPoints;
            bm = this->robot->robotMovement->moveToPoint(query);
            //DriveHelper.DriveToPointAndAlignCareBall(WorldHelper.Allo2Ego(dest, ownPos), ballPos, maxVel, WM);
        }
//		if (mc == nullptr)
//		{
//			return;
//		}
        if (!std::isnan(bm.motion.translation))
        {
            send(bm);
        }
        else
        {
            cout << "motion command is Nan" << endl;
        }
        /*PROTECTED REGION END*/
    }
    void FetchFromSideLine::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1450175655102) ENABLED START*/ //Add additional options here
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        maxVel = (*this->sc)["Behaviour"]->get<double>("Behaviour", "MaxSpeed", NULL);
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1450175655102) ENABLED START*/ //Add additional methods here
    bool FetchFromSideLine::nearSideLine(shared_ptr<geometry::CNPoint2D> alloBall)
    {
        return abs(alloBall->y) > wm->field->getFieldWidth() / 2 - threshold
                && abs(alloBall->y) < wm->field->getFieldWidth() / 2 + threshold;
    }

    bool FetchFromSideLine::nearXLine(shared_ptr<geometry::CNPoint2D> alloBall)
    {
        return abs(alloBall->x) > wm->field->getFieldLength() / 2 - threshold
                && abs(alloBall->y) < wm->field->getFieldLength() / 2 + threshold;
    }

/*PROTECTED REGION END*/
} /* namespace alica */
