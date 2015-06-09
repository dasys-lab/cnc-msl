using namespace std;
#include "Plans/Behaviours/Actuate.h"
#include<iostream>

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

		int left, right;
		// TODO x und y wahrscheinlich durch merge verloren gegangen, nochmal anschauen

		if (rodo == nullptr)
		{
			cout << "Actuate RODO is empty help" << endl;
			return;
		}


		//TODO Ballquality test


		//arithmetic Average for Speed


		double arithmeticAverageSpeed = 0.0;
		double newParamerSpeed = wm->rawSensorData.getOwnVelocityMotion()->translation;
		list<double>::iterator parameterSpeed;

		if (arithmeticAverageBoxSpeed.size() == 3)
		{
			arithmeticAverageBoxSpeed.pop_back();
		}

		arithmeticAverageBoxSpeed.push_front(newParamerSpeed);

		for (parameterSpeed = arithmeticAverageBoxSpeed.begin(); parameterSpeed != arithmeticAverageBoxSpeed.end(); parameterSpeed++)
		{
			arithmeticAverageSpeed += *parameterSpeed;
		}

		arithmeticAverageSpeed = arithmeticAverageSpeed / 3;







		//PIDControllerLeft

		double x = wm->rawSensorData.getOwnVelocityMotion()->angle;
		double righty, lefty, feedForwardLeft, feedForwardRight;
		double KvLeft, KvRight;
		double qualityOfService = wm->rawSensorData.getOpticalFlowQoS();
		double eFunktion = 0.0184 + 0.039637 * exp(-0.003 * arithmeticAverageSpeed);

		righty = 1.8
				* (sin(x - 0.54) - 1 / 9 * sin(3 * (x + 0.16) - 0.2) + 1 / 25 * sin(5 * (x + 0.16))
						- 1 / 49 * sin(7 * (x + 0.16) - 0.1));
		lefty = -1.8
				* (sin(x + 2.58) - 1 / 9 * sin(3 * (x + 3.28) - 0.2) + 1 / 25 * sin(5 * (x + 3.28))
						- 1 / 49 * sin(7 * (x + 3.28) - 0.1));

		//sideward
		//righty = (x * x * 0.6 - x * 0.95 - 1.4);
		//feedForwardRight = max(min(righty, 1.0), -1.6);

		//lefty = (0.6 * x * x + 0.95 * x - 1.4);
		//feedForwardLeft = max(min(lefty, 1.0), -1.6);

		//righty = arithmeticAverage * -x * 1 / 15;
		//lefty = arithmeticAverage * x * 1 / 35;

		//Feedforward
		//Forward
		if (cos(wm->rawSensorData.getOwnVelocityMotion()->angle) < 0)
		{
			if (arithmeticAverageSpeed < 150)
			{
				eFunktion = 0;
			};

			KvRight = (righty * eFunktion * arithmeticAverageSpeed);

			KvLeft = (lefty * eFunktion * arithmeticAverageSpeed);
		};
		//Feedforward
		//Back
			if (cos(wm->rawSensorData.getOwnVelocityMotion()->angle) >= 0)
		 {
		 righty = (wm->rawSensorData.getOwnVelocityMotion()->angle * wm->rawSensorData.getOwnVelocityMotion()->angle
		 * 0.6 - wm->rawSensorData.getOwnVelocityMotion()->angle * 0.95 - 1.4);
		 feedForwardRight = max(min(righty, 1.0), -1.4);
		 KvRight = (righty * eFunktion * arithmeticAverageSpeed - 10);

		 lefty = (0.6 * wm->rawSensorData.getOwnVelocityMotion()->angle
		 * wm->rawSensorData.getOwnVelocityMotion()->angle
		 + 0.95 * wm->rawSensorData.getOwnVelocityMotion()->angle - 1.4);
		 feedForwardLeft = max(min(lefty, 1.0), -1.2);
		 KvLeft = (lefty * eFunktion * arithmeticAverageSpeed - 10);

		 };

		/*
		 //PIDController
		 const double KiLeft = 0.5;
		 const double KdLeft = 0.7;
		 const double KpLeft = 0.23;


		 const double SollwertLeft = 80;


		 double AbweichungLeft = 0.0;
		 double Abweichung_SummeLeft = 0.0;
		 double Abweichung_AltLeft = 0.0;
		 double StellwertLeft = 0.0;

		 AbweichungLeft = -1 * (SollwertLeft - wm->rawSensorData.getOpticalFlowQoS());

		 if (StellwertLeft < 75)
		 Abweichung_SummeLeft += AbweichungLeft;

		 StellwertLeft = KpLeft * AbweichungLeft + KvLeft;
		 StellwertLeft += KiLeft * Abweichung_SummeLeft;
		 StellwertLeft += KdLeft * (AbweichungLeft - Abweichung_AltLeft);

		 Abweichung_AltLeft = AbweichungLeft;


		 //PIDControllerRight


		 const double KiRight = 0.5;
		 const double KdRight = 0.7;
		 const double SollwertRight = 80;
		 const double KpRight = 0.23;

		 double AbweichungRight = 0.0;
		 double Abweichung_SummeRight = 0.0;
		 double Abweichung_AltRight = 0.0;
		 double StellwertRight = 0.0;


		 if (StellwertRight < 75)
		 Abweichung_SummeRight += AbweichungRight;

		 AbweichungRight = -1 * (SollwertRight - wm->rawSensorData.getOpticalFlowQoS());
		 StellwertRight = KpRight * AbweichungRight + KvRight;
		 StellwertRight += KiRight * Abweichung_SummeRight;
		 StellwertRight += KdRight * (AbweichungRight - Abweichung_AltRight);


		 Abweichung_AltRight = AbweichungRight;

		 //StellwertRight /= 4;
		 /*
		 if (Stellwert > 100)
		 Stellwert = 100;
		 if (Stellwert < 80)
		 Stellwert = 80;

		 return Stellwert;



		 cout << "QualityOfService WM : " << wm->rawSensorData.getOpticalFlowQoS() << endl;
		 cout << "Stellwert : " << Stellwert << endl;

		 // left =  rodo->motion.translation * (1.0 / 40.0) ;
		 //right = rodo->motion.translation * (1.0 / 40.0) ;

		 left = -Stellwert;
		 right = -Stellwert;
		 */

		cout << "Winkel : " << x << endl;
		cout << "Speed Approx : " << arithmeticAverageSpeed << " <=> real "
				<< wm->rawSensorData.getOwnVelocityMotion()->translation << endl;

		cout << " QualityOfService : " << wm->rawSensorData.getOpticalFlowQoS() << endl;
		cout << "lefty : " << lefty << endl;
		cout << "KvLeft : " << KvLeft << endl;
		cout << "righty : " << righty << endl;
		cout << "KvRight : " << KvRight << endl << endl;
		cout << endl;
//	cout<<"leftMotor : "<<left<<"   rightStellwert: "<<StellwertRight<<
		//	cout << " cos x :" << cos(x) << endl;

		left =KvLeft; // StellwertLeft;
		right = KvRight; // StellwertRight;

		bhc.leftMotor = max(min(left, 60), -60);
		bhc.rightMotor = max(min(right, 60), -60);
		this->send(bhc);

		/*PROTECTED REGION END*/
	}
	void Actuate::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1417017518918) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
/*PROTECTED REGION ID(methods1417017518918) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
