using namespace std;
#include "Plans/Attack/DribbleToAttackPoint.h"

/*PROTECTED REGION ID(inccpp1436855838589) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "msl_msgs/PathPlanner.h"
#include "msl_msgs/VoronoiNetInfo.h"
#include "pathplanner/evaluator/PathEvaluator.h"
#include "pathplanner/PathProxy.h"
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
		this->maxVel = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.maxVel", NULL);
		this->maxOppDist = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.maxOppDist", NULL);
		this->oppVectorWeight = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.oppVectorWeight", NULL);
		this->clausenDepth = (*this->sc)["Dribble"]->get<int>("DribbleToAttackPoint.clausenDepth", NULL);
		this->clausenPow = (*this->sc)["Dribble"]->get<int>("DribbleToAttackPoint.clausenPow", NULL);
		this->pastRotationSize = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.pastRotationSize", NULL);
		this->driveToWeight = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.driveToWeight", NULL);
		this->maxDribbleSpeed = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.maxDribbleSpeed", NULL);
		voroniPub = n.advertise<msl_msgs::VoronoiNetInfo>("/PathPlanner/VoronoiNet", 10);
		pastRotation.resize(pastRotationSize);
		lastRotError = 0;
		ownPenalty = false;
		counter = -1;

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
		auto vNet = wm->pathPlanner.getCurrentVoronoiNet();
		auto egoBallPos = wm->ball.getEgoBallPosition();
		shared_ptr<geometry::CNPoint2D> egoAlignPoint = nullptr;
		if (ownPos == nullptr || vNet == nullptr)
		{
			cout << "returning" << endl;
			return;
		}
		auto opponents = vNet->getOpponentPositions();
		//    Constant ball handle wheel speed
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
			auto opp = opponents->at(i).first;
			dist = opp->distanceTo(ownPoint);
			if (dist < maxOppDist)
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
			egoAlignPoint = egoTargetPoint;
		}
		else
		{
			closestOpponent = closestOpponent->alloToEgo(*ownPos);
			auto weightedOppVector = closestOpponent->rotate(M_PI) * (1.0 / closestOpponent->length())
					*  (this->oppVectorWeight - closestOpponent->length())/*egoTargetPoint->length() / 5.0*/;
			auto weightedTargetVector = egoTargetPoint * (1.0 / egoTargetPoint->length()) * closestOpponent->length();
			egoAlignPoint = (weightedOppVector + weightedTargetVector)->normalize() * 1000;
		}
		msl_msgs::VoronoiNetInfo netMsg;
		if (closestOpponent != nullptr)
		{
			msl_msgs::Point2dInfo info;
			info.x = closestOpponent->egoToAllo(*ownPos)->x;
			info.y = closestOpponent->egoToAllo(*ownPos)->y;
			netMsg.sites.push_back(info);

//			if (lastClosesOpp != nullptr && (closestOpponent - lastClosesOpp)->length() > 1000)
//			{
//				cout << "changed last closest opp" << endl;
//			}
			lastClosesOpp = closestOpponent;
		}
		msl_actuator_msgs::MotionControl mc;

		shared_ptr<geometry::CNPoint2D> temp = msl::PathProxy::getInstance()->getEgoDirection(egoTargetPoint, eval);
		if (egoAlignPoint->rotate(M_PI)->angleTo() > M_PI / 2)
		{
			mc.motion.rotation = 2 * M_PI;
		}
		else if (egoAlignPoint->rotate(M_PI)->angleTo() < -M_PI / 2)
		{
			mc.motion.rotation = -2 * M_PI;
		}
		else
		{
//        	sinus regelung
//			mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo()
//                    * abs(sin(egoAlignPoint->rotate(M_PI)->angleTo())) * 2; // + (egoAlignPoint->rotate(M_PI)->angleTo() - lastRotError) * 0.3;
//			wurzel regelung
//        	mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo()
//					* sqrt(abs(egoAlignPoint->rotate(M_PI)->angleTo())) * 2;
			double clausenValue = 0.0;
			for (int i = 1; i < this->clausenDepth; i++)
			{
				clausenValue += sin(i * egoAlignPoint->rotate(M_PI)->angleTo()) / pow(i, this->clausenPow);
			}
			double sum = 0;
			for (int i = 0; i < pastRotation.size(); i++)
			{
				sum += pastRotation.at(i);
			}
			mc.motion.rotation = (sum + egoAlignPoint->rotate(M_PI)->angleTo() * abs(clausenValue)) / (this->pastRotationSize + 1); // *4
			counter++;
			pastRotation.at(counter % this->pastRotationSize) = egoAlignPoint->rotate(M_PI)->angleTo() * abs(clausenValue);
		}
		// crate the motion orthogonal to the ball
		shared_ptr<geometry::CNPoint2D> driveTo = egoBallPos->rotate(-M_PI / 2.0);
		driveTo = driveTo * mc.motion.rotation;

		// add the motion towards the ball
		driveTo = driveTo + temp->normalize() * min(maxDribbleSpeed, temp->length()) * this->driveToWeight;

		mc.motion.angle = driveTo->angleTo();
		mc.motion.translation = min(this->maxVel, driveTo->length());
//		cout << "Rotation " << mc.motion.rotation << " Angle " << egoAlignPoint->rotate(M_PI)->angleTo() << endl;
		msl_msgs::Point2dInfo info;
		info.x = egoAlignPoint->egoToAllo(*ownPos)->x;
		info.y = egoAlignPoint->egoToAllo(*ownPos)->y;
		netMsg.sites.push_back(info);
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
		eval = make_shared<msl::PathEvaluator>(&wm->pathPlanner);
		bool success = true;
		string tmp = "";
		success &= getParameter("OwnPenalty", tmp);
		try
		{
			if (success)
			{
				std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
				std::istringstream is(tmp);
				bool b;
				is >> std::boolalpha >> b;
				ownPenalty = b;
			}

		}
		catch (exception& e)
		{
			cerr << "Could not cast the parameter properly" << endl;
		}
		if (!success)
		{
			cerr << "Parameter does not exist" << endl;
		}
		if (!ownPenalty)
		{
			alloTargetPoint = field->posOppPenaltyMarker();
		}
		else
		{
			alloTargetPoint = field->posOwnPenaltyMarker();
		}
		wheelSpeed = -80;
		lastClosesOpp = nullptr;
		lastRotError = 0;
		for (int i = 0; i < pastRotation.size(); i++)
		{
			pastRotation.at(i) = 0;
		}
		counter = -1;
		/*PROTECTED REGION END*/
	}
/*PROTECTED REGION ID(methods1436855838589) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
