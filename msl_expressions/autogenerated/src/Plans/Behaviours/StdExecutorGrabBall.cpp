using namespace std;
#include "Plans/Behaviours/StdExecutorGrabBall.h"

/*PROTECTED REGION ID(inccpp1441209011595) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1441209011595) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	StdExecutorGrabBall::StdExecutorGrabBall() :
			DomainBehaviour("StdExecutorGrabBall")
	{
		/*PROTECTED REGION ID(con1441209011595) ENABLED START*/ //Add additional options here
		readConfigParameters();
		/*PROTECTED REGION END*/
	}
	StdExecutorGrabBall::~StdExecutorGrabBall()
	{
		/*PROTECTED REGION ID(dcon1441209011595) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void StdExecutorGrabBall::run(void* msg)
	{
		/*PROTECTED REGION ID(run1441209011595) ENABLED START*/ //Add additional options here
		geometry::CNPoint2D alloTarget; // alloTarget= the points that robot has to reach.

		shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball.getEgoBallPosition();
		shared_ptr<geometry::CNPosition> me = wm->rawSensorData.getOwnPositionVision();

		MotionControl mc;
		//this->success = true;
		//return;

		/* else
		 {
		 this->success = false;
		 }*/

		// return if necessary information is missing
		if (egoBallPos == nullptr)
		{

			//  (*ownPos).x; ownPos->x smart Pointer

			if (count % 5 == 0)
			{

				alloTarget.x = 0;
				alloTarget.y = 0;

			}
			else if (count % 5 == 1)
			{
				alloTarget.x = -4000;
				alloTarget.y = -4000;
			}
			else if (count % 5 == 2)
			{
				alloTarget.x = -4000;
				alloTarget.y = 4000;
			}
			else if (count % 5 == 3)
			{
				alloTarget.x = 4000;
				alloTarget.y = -4000;
			}

		else
		{
			alloTarget.x = 4000;
			alloTarget.y = 4000;
		}


		auto egoTarget = alloTarget.alloToEgo(*me);

		mc = RobotMovement::moveToPointCarefully(egoTarget, make_shared<geometry::CNPoint2D>(-1000.0, 0.0), 0);

		if (egoTarget->length() < 250)
		{
			count++;
			cout<< "count="<<count<<endl;
		//	cout<< "iteration="<<iteration<<endl;
			if (count %10 ==0)
						{cout <<"no ball in the field"<<endl;}
		}

		send(mc);

		//return;

	}
	else
	{
		MotionControl mc = msl::RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, catchRadius, nullptr);
		send(mc);

	}
	if (wm->ball.haveBall())
	{
	/*	 msl_actuator_msgs::BallHandleCmd bhc;
		if (egoBallPos->length() < 250)
		        {

		            bhc.leftMotor = -30;
		            bhc.rightMotor = -30;

		            this->send(bhc);
		        }*/
		geometry::CNPoint2D allloTarget;
		allloTarget.x = ownPos->x;
		allloTarget.y = ownPos->y;

		auto egolTarget = allloTarget.alloToEgo(*me);
		// 	mc = RobotMovement::moveToPointCarefully(egoTarget,egoTarget, 100);

		double distance = egolTarget->length();
		double movement = kP * distance + kD * (distance - oldDistance);
		oldDistance = distance;

		//mc.motion.translation = movement;
		//mc.motion.angle = egolTarget->angleTo();
	//	mc.motion.rotation = egolTarget->rotate(M_PI)->angleTo() ;
		MotionControl  mc = msl::RobotMovement::moveToPointCarefully(egolTarget,egolTarget, catchRadius, nullptr);
		if (egolTarget->length() < 300)

		{
			this->success = true;
		}

		cout << "mc.motion.angle= " << mc.motion.angle << endl;
		cout << "mc.motion.translation= " << mc.motion.translation << endl;
		cout << "mc.motion.rotation= " << mc.motion.rotation << endl;
		send(mc);
	}

	/*PROTECTED REGION END*/
}
void StdExecutorGrabBall::initialiseParameters()
{
	/*PROTECTED REGION ID(initialiseParameters1441209011595) ENABLED START*/ //Add additional options here
	ownPos = wm->rawSensorData.getOwnPositionVision();
	cout << "ownPos=" << (*ownPos).toString();
	oldDistance = 0.0;
	kP = 0.2;
	kD = 0.1;
	rotate_P = 3;
	isMovingCloserIter = 0;
	isMovingAwayIter = 0;
	maxIter = 4;
	count = 0;
	iteration=0;

	/*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1441209011595) ENABLED START*/ //Add additional methods here
void StdExecutorGrabBall::readConfigParameters()
{
	supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
	catchRadius = (*sc)["Drive"]->get<double>("Drive.Carefully.CatchRadius", NULL);
}
/*PROTECTED REGION END*/
} /* namespace alica */
