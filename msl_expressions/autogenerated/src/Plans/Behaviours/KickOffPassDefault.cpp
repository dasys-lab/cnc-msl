using namespace std;
#include "Plans/Behaviours/KickOffPassDefault.h"

/*PROTECTED REGION ID(inccpp1438778042140) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "engine/model/EntryPoint.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1438778042140) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	KickOffPassDefault::KickOffPassDefault() :
			DomainBehaviour("KickOffPassDefault")
	{
		/*PROTECTED REGION ID(con1438778042140) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	KickOffPassDefault::~KickOffPassDefault()
	{
		/*PROTECTED REGION ID(dcon1438778042140) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void KickOffPassDefault::run(void* msg)
	{
		/*PROTECTED REGION ID(run1438778042140) ENABLED START*/ //Add additional options here
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision();
		shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball.getEgoBallPosition();

		if (ownPos == nullptr || egoBallPos == nullptr)
		{
			return;
		}

		shared_ptr<geometry::CNPoint2D> egoAlignPoint = nullptr;
		EntryPoint* ep = getParentEntryPoint(taskName);
		shared_ptr<geometry::CNPosition> pos = nullptr;
		int id = -1;
		if (ep != nullptr)
		{
			auto parent = this->runningPlan->getParent().lock();
			if (parent == nullptr)
			{
				cout << "parent null" << endl;
				return;
			}
			shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep);
			id = ids->at(0);
			if (id != -1)
			{
				pos = wm->robots.getTeamMatePosition(id);
				egoAlignPoint = make_shared<geometry::CNPoint2D>(pos->x, pos->y);
			}
		}
		else
		{
			shared_ptr<geometry::CNPoint2D> alloAlignPoint = make_shared<geometry::CNPoint2D>(0, 0);
			egoAlignPoint = alloAlignPoint->alloToEgo(*ownPos);
		}

		// Pass message
		shared_ptr<geometry::CNPoint2D> alloPos = make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y);
		msl_helper_msgs::PassMsg pm;
		msl_msgs::Point2dInfo pinf;
		pinf.x = pos->x;
		pinf.y = pos->y;
		pm.destination = pinf;
		pinf.x = alloPos->x;
		pinf.y = alloPos->y;
		pm.origin = pinf;
		pm.receiverID = id;

		double dist = egoAlignPoint->length();
		shared_ptr < geometry::CNPoint2D > dest = make_shared < geometry::CNPoint2D > (-dist, 0);
		dest = dest->egoToAllo(*ownPos);
		shared_ptr<geometry::CNPoint2D> goalReceiverVec = dest - make_shared<geometry::CNPoint2D>(pos->x, pos->y);
		double v0 = 0;
		double distReceiver = goalReceiverVec->length();
		double estimatedTimeForReceiverToArrive = (sqrt(2 * accel * distReceiver + v0 * v0) - v0) / accel;
		pm.validFor = (uint)(estimatedTimeForReceiverToArrive * 1000.0 + 300.0); // this is sparta!!!!!

		msl_actuator_msgs::MotionControl mc = msl::RobotMovement::alignToPointWithBall(egoAlignPoint, egoBallPos, 0.005,
																						0.075);

		// TODO adapt if() so that the robot will shoot when the time is running out
		double angleTolerance = 5;
		if (fabs(egoBallPos->rotate(M_PI)->angleTo()) < (M_PI / 180) * angleTolerance)
		{
			send(mc);
		}
		else
		{
			msl_actuator_msgs::KickControl kc;
			kc.enabled = true;
			kc.kicker = 1;
			kc.power = wm->kicker.getKickPowerPass(egoAlignPoint->alloToEgo(*ownPos)->length());
			send(kc);
			this->success = true;
		}
		/*PROTECTED REGION END*/
	}
	void KickOffPassDefault::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1438778042140) ENABLED START*/ //Add additional options here
		this->accel = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "ReceiverRobotAcceleration", NULL);
		/*PROTECTED REGION END*/
	}
/*PROTECTED REGION ID(methods1438778042140) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
