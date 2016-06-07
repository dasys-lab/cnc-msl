using namespace std;
#include "Plans/TestPlans/GoalieMotionTuning/DriveToPost.h"

/*PROTECTED REGION ID(inccpp1464189819779) ENABLED START*/ //Add additional includes here
#include "RawSensorData.h"
#include "Game.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1464189819779) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DriveToPost::DriveToPost() :
            DomainBehaviour("DriveToPost")
    {
        /*PROTECTED REGION ID(con1464189819779) ENABLED START*/ //Add additional options here
        alignMaxVel = (*sc)["Drive"]->get<double>("Drive", "MaxSpeed", NULL);
        snapDistance = (*this->sc)["Behaviour"]->get<int>("Goalie.SnapDistance", NULL);
        goalieSize = (*this->sc)["Behaviour"]->get<int>("Goalie.GoalieSize", NULL);
        post = (*this->sc)["Behaviour"]->get < string > ("Goalie.PostSide", NULL);

        alloGoalMid = wm->field->posOwnGoalMid();
        alloGoalLeft = make_shared < geometry::CNPoint2D
                > (alloGoalMid->x, wm->field->posLeftOwnGoalPost()->y - goalieSize / 2);
        alloGoalRight = make_shared < geometry::CNPoint2D
                > (alloGoalMid->x, wm->field->posRightOwnGoalPost()->y + goalieSize / 2);

        pTrans = (*this->sc)["Behaviour"]->get<double>("Goalie.pTrans", NULL);
        dTrans = (*this->sc)["Behaviour"]->get<double>("Goalie.dTrans", NULL);

        prevTargetDist = 0;
        startTime = -1;
        /*PROTECTED REGION END*/
    }
    DriveToPost::~DriveToPost()
    {
        /*PROTECTED REGION ID(dcon1464189819779) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DriveToPost::run(void* msg)
    {
        /*PROTECTED REGION ID(run1464189819779) ENABLED START*/ //Add additional options here
        if (wm->game->checkSituation(msl::Situation::Start) && startTime == -1)
        {
            startTime = wm->getTime();
        }
        shared_ptr < geometry::CNPoint2D > targetPost;
        ownPos = wm->rawSensorData->getOwnPositionVision();
        if (post.compare("Left") == 0)
        {
//			cout << "[DriveToPost] driving to left Post.";
            targetPost = alloGoalLeft;
        }
        else if (post.compare("Right") == 0)
        {
//			cout << "[DriveToPost] driving to right Post.";
            targetPost = alloGoalRight;
        }
        else
        {
            cout << "[DriveToPost] no goalPost selected!" << endl;
            return;
        }

        if (targetPost->alloToEgo(*ownPos)->length() > snapDistance)
        {
            cout << "### [DriveToPost] ###" << endl;
            cout << "Remaining distance: " << prevTargetDist << endl;

            ownPos = wm->rawSensorData->getOwnPositionVision();
            mc.motion.angle = targetPost->alloToEgo(*ownPos)->angleTo();
            mc.motion.translation = std::min(
                    alignMaxVel,
                    (targetPost->alloToEgo(*ownPos)->length() * pTrans)
                            + ((targetPost->alloToEgo(*ownPos)->length() - prevTargetDist) * dTrans));
            prevTargetDist = targetPost->alloToEgo(*ownPos)->length();
            cout << endl;
        }
        else
        {
            mc.motion.translation = 0;
            long int endTime = wm->getTime();
            long int time = endTime - startTime;

            if (time > 0)
            {
                cout << "[DriveToPost] Arrived at" << post << "post" << endl;
                cout << "[DriveToPost] startTime: " << startTime << " endTime: " << endTime << endl;
                cout << "[DriveToPost] Time to Post: " << time / 1000000000.0 << endl;
            }
            startTime = -1;
        }
        send (mc);
        /*PROTECTED REGION END*/
    }
    void DriveToPost::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1464189819779) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1464189819779) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
