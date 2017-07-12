using namespace std;
#include "Plans/TestPlans/DribbleControlTest/DribbleToAttackPointTest.h"

/*PROTECTED REGION ID(inccpp1498664309837) ENABLED START*/ //Add additional includes here
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_robot/robotmovement/RobotMovement.h>
//#include "msl_msgs/PathPlanner.h"
//#include "msl_msgs/VoronoiNetInfo.h"
#include "pathplanner/evaluator/PathEvaluator.h"
#include "pathplanner/PathProxy.h"
#include <pathplanner/PathPlanner.h>

/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1498664309837) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	DribbleToAttackPointTest::DribbleToAttackPointTest() :
			DomainBehaviour("DribbleToAttackPointTest")
	{
		/*PROTECTED REGION ID(con1498664309837) ENABLED START*/ //Add additional options here
		alloTargetPoint = make_shared<geometry::CNPoint2D>();
		egoTargetPoint = make_shared<geometry::CNPoint2D>();
		sc = nullptr;
		wheelSpeed = 0;
		lastClosesOpp = make_shared<geometry::CNPoint2D>();
		lastRotError = 0;
		ownPenalty = false;
		pastRotation =
		{};
		counter = 0;
		maxVel = 0;
		maxOppDist = 0;
		maxDribbleSpeed = 0;
		oppVectorWeight = 0;
		clausenDepth = 0;
		clausenPow = 0;
		pastRotationSize = 0;
		orthoDriveWeight = 0;
		targetDriveWeight = 0;

		destinationPoint = make_shared<geometry::CNPoint2D>();
		query = make_shared<msl::MovementQuery>();
		/*PROTECTED REGION END*/
	}
	DribbleToAttackPointTest::~DribbleToAttackPointTest()
	{
		/*PROTECTED REGION ID(dcon1498664309837) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void DribbleToAttackPointTest::run(void* msg)
	{
		/*PROTECTED REGION ID(run1498664309837) ENABLED START*/ //Add additional options here
		auto ownPos = wm->rawSensorData->getOwnPositionVision();
		auto egoBallPos = wm->ball->getEgoBallPosition();
		msl::RobotMovement rm;
		msl_actuator_msgs::MotionControl mc;
		shared_ptr<msl::VoronoiNet> vNet = wm->pathPlanner->getCurrentVoronoiNet();

		if (ownPos == nullptr || egoBallPos == nullptr || vNet == nullptr)
		{
			cerr
					<< "DribbleToAttackPointTest::run(): 'own position' or 'ego ball position' or 'voronoi net' returns null pointer!"
					<< endl;
			return;
		}

		// drive to destination point

		//fill query
		auto egoDestinationPoint = destinationPoint->alloToEgo(*ownPos);
		query->egoDestinationPoint = egoDestinationPoint;

		// get movement command
		mc = rm.moveToPoint(query);

		// turn the robot to the nearest opponent while he is rotating around the ball

		//get opponents
		auto opponents = vNet->getOpponentPositions();
		shared_ptr<geometry::CNPoint2D> egoAlignPoint = nullptr;

		// find nearest opponent
		shared_ptr<geometry::CNPoint2D> closestOpponent = nullptr;
		double lowestDist = numeric_limits<double>::max();
		double dist = 0;
		for (int i = 0; i < opponents->size(); i++)
		{
			auto opp = opponents->at(i);
			dist = opp->distanceTo(ownPos);
			if (dist < maxOppDist)
			{
				if (dist < lowestDist)
				{
					lowestDist = dist;
					closestOpponent = opp;
				}
			}
		}
		//if there is no oppen closer than maxOppDist
		if (closestOpponent == nullptr)
		{
			egoAlignPoint = egoTargetPoint;
		}

		// use rm.alignTo() method
		// try to combine moveToPoint and alignTo

		// send movement command
		if (mc.motion.rotation == NAN || mc.motion.translation == NAN || mc.motion.angle == NAN)
		{
			cerr << "DribbleToAttackPoint::run(): Motion command is NaN!" << endl;
			return;
		}
		send(mc);
		/*PROTECTED REGION END*/
	}
	void DribbleToAttackPointTest::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1498664309837) ENABLED START*/ //Add additional options here
//		wheelSpeed = -80;
//		lastClosesOpp = nullptr;
//		lastRotError = 0;
//		for (int i = 0; i < pastRotation.size(); i++)
//		{
//			pastRotation.at(i) = 0;
//		}
//		counter = -1;
		readConfigParameters();
		destinationPoint = make_shared<geometry::CNPoint2D>(0, 0);
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1498664309837) ENABLED START*/ //Add additional methods here
	void DribbleToAttackPointTest::readConfigParameters()
	{
		this->sc = nullptr;
		this->maxVel = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.maxVel", NULL);
		this->maxOppDist = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.maxOppDist", NULL);
		this->oppVectorWeight = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.oppVectorWeight", NULL);
		this->clausenDepth = (*this->sc)["Dribble"]->get<int>("DribbleToAttackPoint.clausenDepth", NULL);
		this->clausenPow = (*this->sc)["Dribble"]->get<int>("DribbleToAttackPoint.clausenPow", NULL);
		this->pastRotationSize = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.pastRotationSize", NULL);
		this->orthoDriveWeight = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.orthoDriveWeight", NULL);
		this->targetDriveWeight = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.targetDriveWeight", NULL);
		this->maxDribbleSpeed = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.maxDribbleSpeed", NULL);
		pastRotation.resize(pastRotationSize);
	}
/*PROTECTED REGION END*/
} /* namespace alica */
