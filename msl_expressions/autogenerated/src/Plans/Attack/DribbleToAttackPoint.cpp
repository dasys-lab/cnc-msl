using namespace std;
#include "Plans/Attack/DribbleToAttackPoint.h"

/*PROTECTED REGION ID(inccpp1436855838589) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "msl_msgs/PathPlanner.h"
#include "msl_msgs/VoronoiNetInfo.h"
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1436855838589) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	DribbleToAttackPoint::DribbleToAttackPoint() :
			DomainBehaviour("DribbleToAttackPoint")
	{
		/*PROTECTED REGION ID(con1436855838589) ENABLED START*/ //Add additional options here
		this->wheelSpeed = -50;
		this->sc = nullptr;
		this->field = nullptr;
		voroniPub = n.advertise<msl_msgs::VoronoiNetInfo>("/PathPlanner/VoronoiNet", 10);

		/*PROTECTED REGION END*/
	}
	DribbleToAttackPoint::~DribbleToAttackPoint()
	{
		/*PROTECTED REGION ID(dcon1436855838589) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void DribbleToAttackPoint::run(void* msg)
	{
		/*PROTECTED REGION ID(run1436855838589) ENABLED START*/ //Add additional options here
		auto ownPos = wm->rawSensorData.getOwnPositionVision();
		auto egoBallPos = wm->ball.getEgoBallPosition();
//		auto opponents = wm->robots.getObstacles();
		auto vNet = wm->pathPlanner.getCurrentVoronoiNet();
		shared_ptr<geometry::CNPoint2D> egoAlignPoint = nullptr;
		if (ownPos == nullptr || egoBallPos == nullptr || vNet == nullptr)
		{
			return;
		}
		auto opponents = vNet->getOpponentPositions();
		// Constant ball handle wheel speed
		BallHandleCmd bhc;
		bhc.leftMotor = (int8_t)this->wheelSpeed;
		bhc.rightMotor = (int8_t)this->wheelSpeed;
		send(bhc);
		auto ownPoint = make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y);
		egoTargetPoint = alloTargetPoint->alloToEgo(*ownPos);
		shared_ptr<geometry::CNPoint2D> closestOpponent = nullptr;
		double lowestDist = numeric_limits<double>::max();
		double dist = 0;
		for (int i = 0; i < opponents->size(); i++)
		{
//			auto opp = make_shared<geometry::CNPoint2D>(opponents->at(i).x, opponents->at(i).y);
			auto opp = opponents->at(i).first;
			dist = opp->distanceTo(ownPoint);
			if (dist < 3000)
			{
				if (dist < lowestDist)
				{
					lowestDist = dist;
					closestOpponent = opp;
				}
			}
		}
		if (closestOpponent == nullptr)
		{
			cout << "closesOpp == nullptr!" << endl;
			egoAlignPoint = egoTargetPoint;
		}
		else
		{
			closestOpponent = closestOpponent->alloToEgo(*ownPos);
			auto weightedOppVector = closestOpponent->rotate(M_PI) * (1.0 / closestOpponent->length())
					* egoTargetPoint->length();
			auto weightedTargetVector = egoTargetPoint * (1.0 / egoTargetPoint->length()) * closestOpponent->length();
//			egoAlignPoint = (weightedOppVector + weightedTargetVector)->normalize() * 1000;
			egoAlignPoint = weightedOppVector->normalize() * 1000;
		}
		//left = 1
		//right = -1
		int sign = 0;
		msl_msgs::VoronoiNetInfo netMsg;
		if (closestOpponent != nullptr)
		{
			msl_msgs::Point2dInfo info;
			info.x = closestOpponent->egoToAllo(*ownPos)->x;
			info.y = closestOpponent->egoToAllo(*ownPos)->y;
			netMsg.sites.push_back(info);

			if (lastClosesOpp != nullptr && (closestOpponent - lastClosesOpp)->length() > 1000)
			{
				cout << "changed last closest opp" << endl;
			}
			lastClosesOpp = closestOpponent;
		}
		msl_actuator_msgs::MotionControl mc = msl::RobotMovement::moveToPointCarefully(egoTargetPoint, egoAlignPoint,

		250);
		if (abs(egoAlignPoint->rotate(M_PI)->angleTo()) > M_PI / 2)
		{
			mc.motion.rotation = 2 * M_PI;
		}
		else
		{
			mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo() * sin(egoAlignPoint->rotate(M_PI)->angleTo())
					* 2;// + (egoAlignPoint->rotate(M_PI)->angleTo() - lastRotError) * 0.3;
		}
		cout << "Rotation " << mc.motion.rotation << " Angle " << egoAlignPoint->rotate(M_PI)->angleTo() << endl;
		msl_msgs::Point2dInfo info;
		info.x = egoAlignPoint->egoToAllo(*ownPos)->x;
		info.y = egoAlignPoint->egoToAllo(*ownPos)->y;
		netMsg.sites.push_back(info);
		mc.motion.translation = 0;
		if (egoTargetPoint->length() < 250)
		{
			this->success = true;
		}
		lastRotError = egoAlignPoint->rotate(M_PI)->angleTo();
		voroniPub.publish(netMsg);
		send(mc);
		/*PROTECTED REGION END*/
	}
	void DribbleToAttackPoint::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1436855838589) ENABLED START*/ //Add additional options here
		field = msl::MSLFootballField::getInstance();
		sc = supplementary::SystemConfig::getInstance();
		alloTargetPoint = field->posOppPenaltyMarker();
		wheelSpeed = -75;
		lastClosesOpp = nullptr;
		lastRotError = 0;
		/*PROTECTED REGION END*/
	}
/*PROTECTED REGION ID(methods1436855838589) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
