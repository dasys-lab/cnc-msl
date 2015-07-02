using namespace std;
#include "Plans/Behaviours/Actuate.h"

/*PROTECTED REGION ID(inccpp1417017518918) ENABLED START*/ //Add additional includes here
#include "math.h"
#include "RawSensorData.h"

/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1417017518918) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	Actuate::Actuate() :
			DomainBehaviour("Actuate")
	{
		/*PROTECTED REGION ID(con1417017518918) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	Actuate::~Actuate()
	{
		/*PROTECTED REGION ID(dcon1417017518918) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void Actuate::run(void* msg)
	{
		/*PROTECTED REGION ID(run1417017518918) ENABLED START*/ //Add additional options here
		msl_actuator_msgs::BallHandleCmd bhc;
		auto rodo = wm->rawSensorData.getOwnVelocityMotion();

		double left = 0;
		double right = 0;

		if (rodo == nullptr)
		{
			cout << "Actuate RODO is empty help" << endl;
			return;
		}
		newController(left, right);
		//oldController(left,right);

		//PD Regler Anfang
		//PIDControllerLeft
		/*
		 +		const double KiLeft = 0.0;
		 +		const double KdLeft = 0.1;
		 +		const double KpLeft = 0.23;
		 +
		 +		double AbweichungLeft = 0.0;
		 +		double Abweichung_SummeLeft = 0.0;
		 +		double Abweichung_AltLeft = 0.0;
		 +		double StellwertLeft = 0.0;
		 +		const double SollwertLeft = 70;
		 +		//const double SollwertLeftForward = 80;
		 +
		 +		if (wm->rawSensorData.getOpticalFlowQoS() <= 70)
		 +		{
		 +
		 +			Abweichung_SummeLeft += AbweichungLeft;
		 +			AbweichungLeft = -1 * (SollwertLeft - wm->rawSensorData.getOpticalFlowQoS());
		 +			StellwertLeft = KpLeft * AbweichungLeft + KvLeft;
		 +			StellwertLeft += KiLeft * Abweichung_SummeLeft;
		 +			StellwertLeft += KdLeft * (AbweichungLeft - Abweichung_AltLeft);
		 +
		 +			Abweichung_AltLeft = AbweichungLeft;
		 +		};
		 +
		 +		//PIDControllerRight
		 +
		 +		const double KiRight = 0.0;
		 +		const double KdRight = 0.1;
		 +
		 +		const double SollwertRight = 70;
		 +		const double KpRight = 0.23;
		 +
		 +		double AbweichungRight = 0.0;
		 +		double Abweichung_SummeRight = 0.0;
		 +		double Abweichung_AltRight = 0.0;
		 +		double StellwertRight = 0.0;
		 +
		 +		if (wm->rawSensorData.getOpticalFlowQoS() <= 70)
		 +		{
		 +			Abweichung_SummeRight += AbweichungRight;
		 +			AbweichungRight = -1 * (SollwertRight - wm->rawSensorData.getOpticalFlowQoS());
		 +			StellwertRight = KpRight * AbweichungRight + KvRight;
		 +			StellwertRight += KiRight * Abweichung_SummeRight;
		 +			StellwertRight += KdRight * (AbweichungRight - Abweichung_AltRight);
		 +
		 +			Abweichung_AltRight = AbweichungRight;
		 +		};
		 +
		 +		if (wm->rawSensorData.getOpticalFlowQoS() > 70)
		 +		{
		 +			StellwertLeft = KvLeft;
		 +			StellwertRight = KvRight;
		 +		};
		 +
		 +		//PD Regler Ende
		 +*/

		cout << "Winkel : " << wm->rawSensorData.getOwnVelocityMotion()->angle << endl;
		cout << "QualityOfService WM : " << wm->rawSensorData.getOpticalFlowQoS() << endl;
		cout << " rotation : " << wm->rawSensorData.getOwnVelocityMotion()->rotation << endl;

		//cout << "StellwertLeft: " << StellwertLeft << endl;
		//cout << "StellwertRight: " << StellwertRight << endl;
		cout << endl;

		bhc.leftMotor = max(min(left, 60.0), -100.0);
		bhc.rightMotor = max(min(right, 60.0), -100.0);
		this->send(bhc);

		speedDifferenceNew = wm->rawSensorData.getOwnVelocityMotion()->translation;

		/*PROTECTED REGION END*/
	}
	void Actuate::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1417017518918) ENABLED START*/ //Add additional options here
		speedDifference = 0.0;
		double zaeler = 0;
		double qualityOfServiceSumme = 0;

		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1417017518918) ENABLED START*/ //Add additional methods here
	//////////OldController Start
	void Actuate::oldController(double &leftController, double &rightController)
	{

		bool pullNoMatterWhat = false;
		bool controlNoMatterWhat = false;
		bool haveBall = false;
		int itcounter = 0;
		double handlerSpeedFactor = 1.6;
		double speedNoBall = 40;
		double slowTranslation = 100;
		double slowTranslationWheelSpeed = 15;
		double curveRotationFactor = 80;
		double orthoDriveFactor = 0.09;
		double maxhundred = 100;

		double MaxPWM = 90;

		double UpPos = 0;
		double CatchingPos = 100;

		double PassingPos = 28;
		double NormalPos = 90;

		double PingInterval = 200;
		double ShovelSelectRepeatInterval = 40;
		double HaveLightBarrier = 0;

		//speed factor of both wheels dependent on the odometry
		double SpeedFactor = 1.6;

		//speed of wheels if we dont have the ball
		double SpeedNoBall = 40;
		//slow translation of robot, and the speed of the wheels
		double SlowTranslation = 100;

		double SlowTranslationWheelSpeed = 15;
		double CurveRotationFactor = 80;
		double BackwardsSpeed = 52;
		double OrthoDriveFactor = 0.09;

		double LinearFactor = 1.0;
		double UseFactor = false;

		SpeedNoBall = 50;
		SlowTranslation = 200;
		double SlowRotationLeft = 10;
		double SlowRotationRight = 10;

		double RotationLeft = -38;
		double RotationRight = 38;

		BackwardsSpeed = 52;

		double kp = 0.01;
		double ki = 0.01;
		double kd = 0.00008;
		double MinQos = 50;
		double VelocityFactor = 25;
		double VelocityDiff = 0;
		double l = 0, r = 0;
		double orthoL = 0, orthoR = 0;
		double speed = 0;

		// do we have the ball, so that controlling make sense
		/*		haveBall = WorldHelper.HaveBallDribble(WM, WorldHelper.HadBallBefore);

		 if (haveBall && !hadBefore)
		 {
		 itcounter = 0;
		 }
		 */
		//		if (haveBall && itcounter++ < 8)
		//		{
		//			speed = speedNoBall;
		//		}
		if (true)
		{
			// we have the ball to control it, or want to control ignoring the have ball flag, or we tried to pull it for < X iterations

			double speedX = cos(wm->rawSensorData.getOwnVelocityMotion()->angle)
					* wm->rawSensorData.getOwnVelocityMotion()->translation;
			double speedY = sin(wm->rawSensorData.getOwnVelocityMotion()->angle)
					* wm->rawSensorData.getOwnVelocityMotion()->translation;
			//langsam vorwaerts
			if (speedX > -slowTranslation && speedX < 40)
			{
				speed = slowTranslationWheelSpeed;
			}
			//langsam rueckwaerts
			else if (speedX < slowTranslation && speedX >= 40)
			{
				speed = -slowTranslationWheelSpeed;
			}
			//schnell vor
			else if (speedX <= -slowTranslation)
			{
				double minSpeedOne = handlerSpeedFactor * speedX / 100.0;
				speed = max(-maxhundred, min(maxhundred, minSpeedOne));
			}
			//schnell rueck
			else
			{
				double minSpeedTwo = 3 * handlerSpeedFactor * speedX / 100.0;
				speed = max(-maxhundred, min(maxhundred, minSpeedTwo));
			}

			//geschwindigkeitsanteil fuer orthogonal zum ball
			if (speedY > 0)
			{
				//nach rechts fahren
				orthoR = speedY * orthoDriveFactor;
				orthoL = -speedY * orthoDriveFactor / 2.0;
			}
			else
			{
				//nach links fahren
				orthoR = speedY * orthoDriveFactor / 2.0;
				orthoL = -speedY * orthoDriveFactor;
			}

			//geschwindigkeitsanteil fuer rotation
			double rotation = wm->rawSensorData.getOwnVelocityMotion()->rotation;
			if (rotation < 0)
			{
				l = 0;
				r = abs(rotation) / M_PI * curveRotationFactor;
			}
			else
			{
				r = 0;
				l = abs(rotation) / M_PI * curveRotationFactor;
			}

		}
		else if (!haveBall)
		{
			// we don't have the ball
			speed = speedNoBall;
		}

		double minSpeedThreeLeft = speed + l + orthoL;
		double minSpeedThreeRight = speed + r + orthoR;
		double KvLeft;
		double KvRight;

		KvLeft = -1.0 * max(-maxhundred, min(maxhundred, minSpeedThreeLeft));
		KvRight = -1.0 * max(-maxhundred, min(maxhundred, minSpeedThreeRight));

		cout << "OldController " << endl;
		cout << "KvRight : " << KvRight << endl;
		cout << "KvLeft : " << KvLeft << endl;

		leftController = KvLeft;
		rightController = KvRight;
		//////////OldController End
	}

	void Actuate::newController(double &leftController, double &rightController)
	{
		///////////////////////////////////////////////////////////////////////
		//New Controller Start

		cout << "NewController " << endl;

		//arithmetic Average for Speed Start

		double arithmeticAverageSpeed = 0.0;
		double newParamerSpeed = wm->rawSensorData.getOwnVelocityMotion()->translation;
		//double wtf = wm->rawSensorData.getLastMotionCommand()->motion;

		if (arithmeticAverageBoxSpeed.size() == 2)
		{
			arithmeticAverageBoxSpeed.pop_back();
		}

		arithmeticAverageBoxSpeed.push_front(newParamerSpeed);

		for (list<double>::iterator parameterSpeed = arithmeticAverageBoxSpeed.begin();
				parameterSpeed != arithmeticAverageBoxSpeed.end(); parameterSpeed++)
		{
			arithmeticAverageSpeed += *parameterSpeed;
		}

		arithmeticAverageSpeed = arithmeticAverageSpeed / 2;

		cout << "Speed Approx : " << arithmeticAverageSpeed << " <=> real "
				<< wm->rawSensorData.getOwnVelocityMotion()->translation << endl;

		//arithmetic Average for Speed End

		//Speed Difference for acceleration Start
		double eFunktionAcceleration;
		double newSpeed = wm->rawSensorData.getOwnVelocityMotion()->translation;
		speedDifference = newSpeed - speedDifferenceNew;
		speedDifference = (speedDifference) / 300;

		if (speedDifference < 1)
		{
			speedDifference = 1;
		}

		//  cout << "speedDifference : " << speedDifference << endl;

		//Speed Difference for acceleration End

		////arithmetic average speed difference Start
		double arithmeticAverageSpeedDifference = 0.0;

		if (arithmeticAverageBoxSpeedDifference.size() == 5)
		{
			arithmeticAverageBoxSpeedDifference.pop_back();
		}

		arithmeticAverageBoxSpeedDifference.push_front(speedDifference);

		for (list<double>::iterator parameterSpeedDifference = arithmeticAverageBoxSpeedDifference.begin();
				parameterSpeedDifference != arithmeticAverageBoxSpeedDifference.end(); parameterSpeedDifference++)
		{
			arithmeticAverageSpeedDifference = arithmeticAverageSpeedDifference + *parameterSpeedDifference;
		}

		arithmeticAverageSpeedDifference = arithmeticAverageSpeedDifference / 5;

		if (arithmeticAverageSpeedDifference < 1)
		{
			arithmeticAverageSpeedDifference = 1;
		}
		//  cout << "arithmeticAverageSpeedDifference : " << arithmeticAverageSpeedDifference << endl;

		////arithmetic average speed difference End

		//Rotation Controller Start

		double rotationLeft = 0.0;
		double rotationRight = 0.0;

		/*
		 if ((rotation < -0.3) || (rotation > 0.3))
		 {

		 rotationLeft = (-25 / M_PI * (atan(rotation / 0.001) + M_PI / 2))
		 - 7 / M_PI * (atan(-rotation / 0.001) + M_PI / 2);

		 rotationLeft = rotationLeft - 70;

		 rotationRight = (-7 / M_PI * (atan(rotation / 0.001) + M_PI / 2))
		 - 25 / M_PI * (atan(-rotation / 0.001) + M_PI / 2);

		 rotationRight = rotationRight - 70;
		 };*/

		if (wm->rawSensorData.getOwnVelocityMotion()->rotation > 0.2)
		{

			rotationLeft = -wm->rawSensorData.getOwnVelocityMotion()->rotation * 59;
			rotationRight = -wm->rawSensorData.getOwnVelocityMotion()->rotation * 33;
			cout << "rotation	left : " << rotationLeft << endl;
			cout << "rotation right : " << rotationRight << endl;

		}

		if (wm->rawSensorData.getOwnVelocityMotion()->rotation < -0.2)
		{

			rotationRight = wm->rawSensorData.getOwnVelocityMotion()->rotation * 59;
			rotationLeft = wm->rawSensorData.getOwnVelocityMotion()->rotation * 33;

			cout << "rotation	left : " << rotationLeft << endl;
			cout << "rotation right : " << rotationRight << endl;

		}

		//Rotation Controller End

		//Exp Funktion for traction Start

		double feedForwardLeft, feedForwardRight;
		double KvLeft, KvRight;
		double x;
		double angle = wm->rawSensorData.getOwnVelocityMotion()->angle;

		double funktionLeft, funktionRight;
		double qualityOfService = wm->rawSensorData.getOpticalFlowQoS();
		double eFunktion = 0.0184 + 0.039637 * exp(-0.003 * arithmeticAverageSpeed);

		cout << "exp Funktion : " << eFunktion << endl;
		//Exp Funktion for traction End

		//Funktion for drive with differt angles start

		x = max(min(angle, 3.14), -3.14);

		funktionLeft = 1.0
				* (0.00337 * pow(x, 8) - 0.00154 * pow(x, 7) - 0.0756 * pow(x, 6) + 0.0036 * pow(x, 5)
						+ 0.5517 * pow(x, 4) + 0.0489 * pow(x, 3) - 0.987 * pow(x, 2) + 0.637 * x - 2.292);

		funktionRight = 1.0
				* (0.00337 * pow(x, 8) + 0.00154 * pow(x, 7) - 0.0756 * pow(x, 6) - 0.0036 * pow(x, 5)
						+ 0.5517 * pow(x, 4) - 0.0489 * pow(x, 3) - 0.987 * pow(x, 2) - 0.637 * x - 2.292);

		KvRight = (1.1 * eFunktion * arithmeticAverageSpeed * funktionRight + rotationRight);

		KvLeft = (1.1 * eFunktion * arithmeticAverageSpeed * funktionLeft + rotationLeft);
		cout << "funktionLeft : " << funktionLeft << endl;
		cout << "funktionRight : " << funktionRight << endl;
		cout << "KvLeft : " << KvLeft << endl;
		cout << "KvRight : " << KvRight << endl;

		//Funktion for drive with differt angles end
		leftController = KvLeft;
		rightController = KvRight;

		//Sensor calibration Start

		double teiler;
		qualityOfServiceSumme = qualityOfServiceSumme + wm->rawSensorData.getOpticalFlowQoS();
		zaeler = zaeler + 1;
		teiler = qualityOfServiceSumme / zaeler;

		//	cout << "qualityOfServiceSumme  : " << qualityOfServiceSumme << endl;
		//	cout << "zaeler  : " << zaeler << endl;
		//	cout << "teiler  : " << teiler << endl;

		//Sensor calibration End

	}

/*PROTECTED REGION END*/
} /* namespace alica */
