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

		//Speed Difference for acceleration
		double eFunktionAcceleration;
		double newSpeed = wm->rawSensorData.getOwnVelocityMotion()->translation;
		speedDifference = newSpeed - speedDifferenceNew;
		speedDifference=(speedDifference)/100;

		if (speedDifference < 1){
			speedDifference = 1;
		}

		////arithmetic average speed difference
		double arithmeticAverageSpeedDifference = 0.0;

				if (arithmeticAverageBoxSpeedDifference.size() == 5)
				{
					arithmeticAverageBoxSpeedDifference.pop_back();
				}

				arithmeticAverageBoxSpeedDifference.push_front(speedDifference);

				for (list<double>::iterator parameterSpeedDifference = arithmeticAverageBoxSpeedDifference.begin();
						parameterSpeedDifference != arithmeticAverageBoxSpeedDifference.end(); parameterSpeedDifference++)
				{
					arithmeticAverageSpeedDifference =arithmeticAverageSpeedDifference+*parameterSpeedDifference;
				}

				arithmeticAverageSpeedDifference = arithmeticAverageSpeedDifference / 5;

				if (arithmeticAverageSpeedDifference < 1){
					arithmeticAverageSpeedDifference = 1;
				}


		/////////////////////////////////////////////


	//PIDControllerLeft


		double funktionLeftLim,funktionRightLim, feedForwardLeft, feedForwardRight;
		double KvLeft, KvRight;
		double x;
		double angle= wm->rawSensorData.getOwnVelocityMotion()->angle;

		double funktionLeft,funktionRight;
		double qualityOfService = wm->rawSensorData.getOpticalFlowQoS();
		double eFunktion = 0.0184 + 0.039637 * exp(-0.003 * arithmeticAverageSpeed);
		/*
		 *
		 * 	//gescheiterte Funktionen

		 righty = 1.8

		 * (sin(-x - 0.52) - 1 / 9 * sin(3 * (-x + 0.18) - 0.2) + 1 / 25 * sin(5 * (-x + 0.18))
		 - 1 / 49 * sin(7 * (-x + 0.18) - 0.1));
		 ;
		 lefty = 1.8
		 * (sin(x - 0.52) - 1 / 9 * sin(3 * (x + 0.18) - 0.2) + 1 / 25 * sin(5 * (x + 0.18))
		 - 1 / 49 * sin(7 * (x + 0.18) - 0.1));


		 //sideward
		 righty = 0.021*x*x*x*x +0.065*x*x*x+0.148*x*x-0.48*x-2;
		 feedForwardRight = max(min(righty, 1.0), -2.0);

		 lefty = 0.021*x*x*x*x +0.065*x*x*x+0.148*x*x+0.48*x-2;;
		 feedForwardLeft = max(min(lefty, 1.0), -2.0);

		 */

		x = max(min(angle, 3.14), -3.14);


		funktionLeft=0.00337*pow(x,8)-0.00154*pow(x,7) -0.0756*pow(x,6) +0.0036*pow(x,5)+0.5517*pow(x,4)+0.0489*pow(x,3)-0.987*pow(x,2)+0.637*x-2.292;

		funktionRight=0.00337*pow(x,8)+0.00154*pow(x,7)-0.0756*pow(x,6)-0.0036*pow(x,5)+0.5517*pow(x,4)-0.0489*pow(x,3)-0.987*pow(x,8)-0.637*x-2.292;


		//funktionLeftLim = max(min(funktionLeft, 1.0), -3.5);
		//funktionRightLim = max(min(funktionRight,1.0), -3.5);



		/*
		 //Feedforward
		 //Forward
		 //			if (cos(wm->rawSensorData.getOwnVelocityMotion()->angle) < 0)
		 //			{

		 */
		KvRight =(eFunktion *arithmeticAverageSpeed*funktionRight);

		KvLeft =(eFunktion * arithmeticAverageSpeed*funktionLeft);
//			};
		//Feedforward
		//Back
		/*
		 if (cos(wm->rawSensorData.getOwnVelocityMotion()->angle) >= 0)
		 {

		 KvRight = ( eFunktion * arithmeticAverageSpeed - 10);
		 KvLeft = ( eFunktion * arithmeticAverageSpeed - 10);

		 };
*/
		 //PIDControllerLeft
	/*	 const double KiLeft = 0.4;
		 const double KdLeft = 0.5;
		 const double KpLeft = 0.23;




		 double AbweichungLeft = 0.0;
		 double Abweichung_SummeLeft = 0.0;
		 double Abweichung_AltLeft = 0.0;
		 double StellwertLeft = 0.0;
		 const double SollwertLeftForward = 80;
		 const double SollwertLeftBackward = 50;

		 if ((StellwertLeft < 50)&&(cos(wm->rawSensorData.getOwnVelocityMotion()->angle)>=0))
		 {

		 Abweichung_SummeLeft += AbweichungLeft;
		 AbweichungLeft = -1 * (SollwertLeftBackward - wm->rawSensorData.getOpticalFlowQoS());
		 StellwertLeft = KpLeft * AbweichungLeft + KvLeft;
		 StellwertLeft += KiLeft * Abweichung_SummeLeft;
		 StellwertLeft += KdLeft * (AbweichungLeft - Abweichung_AltLeft);

		 Abweichung_AltLeft = AbweichungLeft;
		 };

		 if ((StellwertLeft < 80)&&(cos(wm->rawSensorData.getOwnVelocityMotion()->angle)<0))
		 {

		 Abweichung_SummeLeft += AbweichungLeft;
		 AbweichungLeft = -1 * (SollwertLeftForward - wm->rawSensorData.getOpticalFlowQoS());
		 StellwertLeft = KpLeft * AbweichungLeft + KvLeft;
		 StellwertLeft += KiLeft * Abweichung_SummeLeft;
		 StellwertLeft += KdLeft * (AbweichungLeft - Abweichung_AltLeft);

		 Abweichung_AltLeft = AbweichungLeft;
		 };

		 //PIDControllerRight

		 const double KiRight = 0.4;
		 const double KdRight = 0.5;
		 const double SollwertRightForward = 80;
		 const double SollwertRightBackward = 50;
		 const double KpRight = 0.23;

		 double AbweichungRight = 0.0;
		 double Abweichung_SummeRight = 0.0;
		 double Abweichung_AltRight = 0.0;
		 double StellwertRight = 0.0;

		 if ((StellwertRight < 50)&&(cos(wm->rawSensorData.getOwnVelocityMotion()->angle)>=0))
		 {
		 Abweichung_SummeRight += AbweichungRight;
		 AbweichungRight = -1 * (SollwertRightBackward - wm->rawSensorData.getOpticalFlowQoS());
		 StellwertRight = KpRight * AbweichungRight + KvRight;
		 StellwertRight += KiRight * Abweichung_SummeRight;
		 StellwertRight += KdRight * (AbweichungRight - Abweichung_AltRight);

		 Abweichung_AltRight = AbweichungRight;
		 };

		 if ((StellwertRight < 80)&&(cos(wm->rawSensorData.getOwnVelocityMotion()->angle)<0))
		 {
		 Abweichung_SummeRight += AbweichungRight;


		 AbweichungRight = -1 * (SollwertRightForward - wm->rawSensorData.getOpticalFlowQoS());
		 StellwertRight = KpRight * AbweichungRight + KvRight;
		 StellwertRight += KiRight * Abweichung_SummeRight;
		 StellwertRight += KdRight * (AbweichungRight - Abweichung_AltRight);

		 Abweichung_AltRight = AbweichungRight;
		 };

		 //StellwertRight /= 4;
		 /*
		 if (Stellwert > 100)
		 Stellwert = 100;
		 if (Stellwert < 80)
		 Stellwert = 80;

		 return Stellwert;




		 cout << "Stellwert : " << Stellwert << endl;

		 // left =  rodo->motion.translation * (1.0 / 40.0) ;
		 //right = rodo->motion.translation * (1.0 / 40.0) ;

		 left = -Stellwert;
		 right = -Stellwert;
		 */

		//nur test danach löschen!!
		//	KvLeft=-1.5*x*KvLeft;
		//	KvRight=-2.5*x*KvRight;
		cout << "Winkel : " << wm->rawSensorData.getOwnVelocityMotion()->angle << endl;
		cout << "Speed Approx : " << arithmeticAverageSpeed << " <=> real "
				<< wm->rawSensorData.getOwnVelocityMotion()->translation << endl;
		// cout << "QualityOfService WM : " << wm->rawSensorData.getOpticalFlowQoS() << endl;
		//cout << "funktionLeftLim : " << funktionLeftLim << endl;
		cout << "KvLeft : " << KvLeft << endl;
		//cout << "StellwertLeft: " << StellwertLeft << endl;
		//cout << "funktionRightLim : " << funktionRightLim << endl;
		cout << "KvRight : " << KvRight << endl;
		cout << "funktionLeft : " << funktionLeft << endl;
		cout << "funktionRight : " << funktionRight << endl;

		//cout << "StellwertRight: " << StellwertRight << endl;
		//cout << "speedDifference : " << speedDifference << endl;
	//	cout << "arithmeticAverageSpeedDifference : " << arithmeticAverageSpeedDifference << endl;

		cout << endl;
//	cout<<"leftMotor : "<<left<<"   rightStellwert: "<<StellwertRight<<
		//	cout << " cos x :" << cos(x) << endl;

		left =  KvLeft; //StellwertLeft;//
		right = KvRight; //StellwertRight;//

		bhc.leftMotor = max(min(left, 60), -80);
		bhc.rightMotor = max(min(right, 60), -80);
		this->send(bhc);

		speedDifferenceNew =wm->rawSensorData.getOwnVelocityMotion()->translation;


		/*PROTECTED REGION END*/
	}
	void Actuate::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1417017518918) ENABLED START*/ //Add additional options here
		speedDifference = 0.0;

		/*PROTECTED REGION END*/
	}
/*PROTECTED REGION ID(methods1417017518918) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
