using namespace std;
#include "Plans/Attack/FetchFromSideLine.h"

/*PROTECTED REGION ID(inccpp1450175655102) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
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
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision(); //OwnPositionCorrected;
        if (ownPos == nullptr)
        {
            this->failure = true;
            return;
        }

        shared_ptr < geometry::CNPoint2D > ballPos = wm->ball.getEgoBallPosition();
        if (ballPos == nullptr)
        {
            this->failure = true;
            return;
        }
        msl_actuator_msgs::MotionControl bm = msl::RobotMovement::ruleActionForBallGetter();

        if (bm.senderID == -1)
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
                    this->success = true;
                }
                dest->x = alloBall->x;
                dest->y = alloBall->y - behindDistance;
            }
            else
            {
                if (ownPos->y > alloBall->y + behindDistance)
                {
                    this->success = true;
                }
                dest->x = alloBall->x;
                dest->y = alloBall->y + behindDistance;
            }
            shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
                    vector<shared_ptr<geometry::CNPoint2D>>>();
            additionalPoints->push_back(alloBall);
            bm = msl::RobotMovement::moveToPointCarefully(dest->alloToEgo(*ownPos), dest->alloToEgo(*ownPos), 0,
                                                          additionalPoints);
            //DriveHelper.DriveToPointAndAlignCareBall(WorldHelper.Allo2Ego(dest, ownPos), ballPos, maxVel, WM);
        }
        if (nearXLine (alloBall))
        {
            if (alloBall->x < 0)
            {
                if (ownPos->x < alloBall->x - behindDistance)
                {
                    this->success = true;
                }
                dest->x = alloBall->x - behindDistance;
                dest->y = alloBall->y;
            }
            else
            {
                if (ownPos->x > alloBall->x + behindDistance)
                {
                    this->success = true;
                }
                dest->x = alloBall->x + behindDistance;
                dest->y = alloBall->y;
            }
            shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
                    vector<shared_ptr<geometry::CNPoint2D>>>();
            additionalPoints->push_back(alloBall);
            bm = msl::RobotMovement::moveToPointCarefully(dest->alloToEgo(*ownPos), dest->alloToEgo(*ownPos), 0,
                                                          additionalPoints);
            //DriveHelper.DriveToPointAndAlignCareBall(WorldHelper.Allo2Ego(dest, ownPos), ballPos, maxVel, WM);
        }
//		if (mc == nullptr)
//		{
//			return;
//		}
        send(bm);
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
        return abs(alloBall->y) > wm->field.getFieldWidth() / 2 - threshold
                && abs(alloBall->y) < wm->field.getFieldWidth() / 2 + threshold;
    }

    bool FetchFromSideLine::nearXLine(shared_ptr<geometry::CNPoint2D> alloBall)
    {
        return abs(alloBall->x) > wm->field.getFieldLength() / 2 - threshold
                && abs(alloBall->y) < wm->field.getFieldLength() / 2 + threshold;
    }

/*PROTECTED REGION END*/
} /* namespace alica */
