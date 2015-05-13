using namespace std;
#include "Plans/Penalty/PenaltyAlignAndShoot.h"

/*PROTECTED REGION ID(inccpp1431531496053) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1431531496053) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	PenaltyAlignAndShoot::PenaltyAlignAndShoot() :
			DomainBehaviour("PenaltyAlignAndShoot")
	{
		/*PROTECTED REGION ID(con1431531496053) ENABLED START*/ //Add additional options here
		this->setTrigger(&wm->visionTrigger);
		field = msl::MSLFootballField::getInstance();
		maxVel = (*this->sc)["Penalty"]->get<double>("Penalty.MaxSpeed", NULL);

		// Aiming/Rotation Stuff
		lastRotError = 0;
		//timesOnTargetThreshold = (*this->sc)["Show"]->get<int>("TwoHoledWall.TimesOnTarget", NULL);
		pRot = (*this->sc)["Penalty"]->get<double>("Penalty.RotationP", NULL);
		dRot = (*this->sc)["Penalty"]->get<double>("Penalty.RotationD", NULL);
		minRot = (*this->sc)["Penalty"]->get<double>("Penalty.MinRotation", NULL);
		maxRot = (*this->sc)["Penalty"]->get<double>("Penalty.MaxRotation", NULL);
		angleTolerance = (*this->sc)["Penalty"]->get<double>("Penalty.AngleTolerance", NULL);
		ballAngleTolerance = (*this->sc)["Penalty"]->get<double>("Penalty.BallAngleTolerance", NULL);
		ballDiameter = (*this->sc)["Globals"]->get<double>("Globals.Dimensions.DiameterBall", NULL);
		goalLineLength = (*this->sc)["Globals"]->get<double>("Globals.FootballField.GoalWidth", NULL);
		robotDiameter = (*this->sc)["Globals"]->get<double>("Globals.Dimensions.DiameterRobot", NULL);

		/*PROTECTED REGION END*/
	}
	PenaltyAlignAndShoot::~PenaltyAlignAndShoot()
	{
		/*PROTECTED REGION ID(dcon1431531496053) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void PenaltyAlignAndShoot::run(void* msg)
	{
		/*PROTECTED REGION ID(run1431531496053) ENABLED START*/ //Add additional options here
		shared_ptr<CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision(); // actually ownPosition corrected
		shared_ptr<CNPoint2D> egoBallPos = wm->ball.getEgoBallPosition();

		// stupid variant to be sure, that we have shot!!!
//		if (kicked)
//		{
//			this->iterationsAfterKick++;
//			if (iterationsAfterKick > 30 && egoBallPos != nullptr && egoBallPos->length() <= 400)
//			{
//				kicked = false;
//				iterationsAfterKick = 0;
//			}
//
//			if (egoBallPos == nullptr || egoBallPos->length() > 400)
//			{
//				if (holeMode == toggle)
//				{
//					useLowerHole = !useLowerHole;
//				}
//				this->success = true;
//			}
//
//			return;
//		}

		if (ownPos == nullptr || egoBallPos == nullptr)
		{
			return;
		}

		// Constant ball handle wheel speed
//		BallHandleCmd bhc;
//		bhc.leftMotor = (int8_t)this->wheelSpeed;
//		bhc.rightMotor = (int8_t)this->wheelSpeed;
//		send(bhc);

		// Create ego-centric 2D target...
//		shared_ptr<CNPoint2D> egoHole;
//		CNPoint2D alloHole(higherHole.x, higherHole.y);
//		if (useLowerHole)
//		{
//			alloHole.x = lowerHole.x;
//			alloHole.y = lowerHole.y;
//		}
//		egoHole = alloHole.alloToEgo(*ownPos);
//
//		double egoHoleAngle = egoHole->angleTo();
//		double egoBallAngle = egoBallPos->angleTo();
//		double deltaHoleAngle = GeometryCalculator::deltaAngle(egoHoleAngle, M_PI);
//		double deltaBallAngle = GeometryCalculator::deltaAngle(egoBallAngle, M_PI);

		// Counter for correct aiming
		//		if (fabs(deltaHoleAngle) < this->angleTolerance)
//		if (fabs(deltaBallAngle) < this->ballAngleTolerance && fabs(deltaHoleAngle) < this->angleTolerance)
//		{
//			//cout << "align and shoot: hit target" << endl;
//			timesOnTargetCounter++;
//		}
//		else
//		{
//			//cout << "align and shoot: miss target" << endl;
//			timesOnTargetCounter = 0;
//		}

		// Kick if aiming was correct long enough

		KickControl kc;
		kc.enabled = true;
		kc.kicker = egoBallPos->angleTo();
//		kc.power = setKickPower(egoHole->length());
		float voltage;
//		if (!disableKicking)
//		{
//			send(kc);
//			kicked = true;
//			iterationsAfterKick = 0;
//			voltage = wm->getKickerVoltage();
//		}
//		else
//		{
//			// Send stop message to motion, in order to signal that the robot would shoot now
//			MotionControl empty;
//			send(empty);
//			return;
//		}


	// Create Motion Command for aiming
	MotionControl mc;

	// PD Rotation Controller
//	mc.motion.rotation = -(deltaHoleAngle * pRot + (deltaHoleAngle - lastRotError) * dRot);
//	mc.motion.rotation = (mc.motion.rotation < 0 ? -1 : 1)
//	* min(this->maxRot, max(fabs(mc.motion.rotation), this->minRot));
//
//	lastRotError = deltaHoleAngle;

	// crate the motion orthogonal to the ball
	shared_ptr<CNPoint2D> driveTo = egoBallPos->rotate(-M_PI / 2.0);
	driveTo = driveTo * mc.motion.rotation;

	// add the motion towards the ball
	driveTo = driveTo + egoBallPos->normalize() * 10;

	mc.motion.angle = driveTo->angleTo();
	mc.motion.translation = min(this->maxVel, driveTo->length());

	send (mc);
/*PROTECTED REGION END*/
}
void PenaltyAlignAndShoot::initialiseParameters()
{
	/*PROTECTED REGION ID(initialiseParameters1431531496053) ENABLED START*/ //Add additional options here


	/*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1431531496053) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
