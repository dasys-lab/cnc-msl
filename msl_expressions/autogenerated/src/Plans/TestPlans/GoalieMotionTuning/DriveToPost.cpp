using namespace std;
#include "Plans/TestPlans/GoalieMotionTuning/DriveToPost.h"

/*PROTECTED REGION ID(inccpp1464189819779) ENABLED START*/ //Add additional includes here
#include "RawSensorData.h"
#include "Game.h"
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
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

		alloGoalMid = wm->field->posOwnGoalMid();
		alloGoalLeft = make_shared<geometry::CNPoint2D>(alloGoalMid->x,
														wm->field->posLeftOwnGoalPost()->y - goalieSize / 2);
		alloGoalRight = make_shared<geometry::CNPoint2D>(alloGoalMid->x,
															wm->field->posRightOwnGoalPost()->y + goalieSize / 2);

		pTrans = (*this->sc)["Behaviour"]->get<double>("Goalie.pTrans", NULL);
		dTrans = (*this->sc)["Behaviour"]->get<double>("Goalie.dTrans", NULL);

		prevTargetDist = 0;
		startTime = -1;
		avgTime = 0.0;

		// 0 stands for left post 1 stands for right post
		driveToPost = 0;
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
		shared_ptr<geometry::CNPoint2D> targetPost;
		ownPos = wm->rawSensorData->getOwnPositionVision();

		if (driveToPost == 0)
		{
			targetPost = alloGoalLeft;
		}
		else if (driveToPost == 1)
		{
			targetPost = alloGoalRight;
		}

		if (targetPost->alloToEgo(*ownPos)->length() > snapDistance)
		{
			ownPos = wm->rawSensorData->getOwnPositionVision();
			mc.motion.angle = targetPost->alloToEgo(*ownPos)->angleTo();
			mc.motion.translation = std::min(
					alignMaxVel,
					(targetPost->alloToEgo(*ownPos)->length() * pTrans)
							+ ((targetPost->alloToEgo(*ownPos)->length() - prevTargetDist) * dTrans));
			prevTargetDist = targetPost->alloToEgo(*ownPos)->length();
		}
		else
		{
			mc.motion.translation = 0;
			long int time = (wm->getTime() - startTime);

			if (time > 1000000000)
			{
				if(avgTime == 0) {
					avgTime = time;
				}
				avgTime = (avgTime + time) / 2.0;
				cout << "[DriveToPost]    Time : " << time / 1000000000.0 << endl;
				cout << "[DriveToPost] AvgTime : " << avgTime / 1000000000.0 << endl;
			}

			// change target to other post side
			if (driveToPost == 1)
			{
				driveToPost--;
			}
			else
			{
				driveToPost++;
			}
			startTime = -1;
		}
		send(mc);
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
