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
		//Function for Left

		//PIDControllerLeft
		double x, lefty, righty, feedForwardLeft, feedForwardRight;
		double KpRight, KpLeft;
		double qualityOfService = wm->rawSensorData.getOpticalFlowQoS();
		x = wm->rawSensorData.getOwnVelocityMotion()->angle;

		lefty = 1.2*(x * x * 0.6 - x * 0.95 - 1.2);
		feedForwardLeft = max(min(lefty, 1.0), -1.5);

		KpLeft = feedForwardLeft * wm->rawSensorData.getOwnVelocityMotion()->translation * 1 / 37;

		//Function for Right

		righty = 1.2*(0.6 * x * x + 0.95 * x - 1.2);

		feedForwardRight = max(min(righty, 1.0), -1.5);

		KpRight = feedForwardRight * wm->rawSensorData.getOwnVelocityMotion()->translation * 1 / 37;

		/*
		 const double KiLeft = 0.0;
		 const double KdLeft = 0.0;
		 const double SollwertLeft = 90;


		 double AbweichungLeft = 0.0;
		 double Abweichung_SummeLeft = 0.0;
		 double Abweichung_AltLeft = 0.0;
		 double StellwertLeft = 0.0;

		 AbweichungLeft = SollwertLeft - wm->rawSensorData.getOpticalFlowQoS();

		 if ((StellwertLeft < 70) || (StellwertLeft > 100))
		 Abweichung_SummeLeft += AbweichungLeft;

		 StellwertLeft = KpLeft * AbweichungLeft;
		 StellwertLeft += KiLeft * Abweichung_SummeLeft;
		 StellwertLeft += KdLeft * (AbweichungLeft - Abweichung_AltLeft);

		 Abweichung_AltLeft = AbweichungLeft;

		 //StellwertLeft /= 4;











		 //PIDControllerRight

		 const double KiRight = 0.0;
		 const double KdRight = 0.0;
		 const double SollwertRight = 90;


		 double AbweichungRight = 0.0;
		 double Abweichung_SummeRight = 0.0;
		 double Abweichung_AltRight = 0.0;
		 double StellwertRight = 0.0;

		 AbweichungRight = SollwertRight - wm->rawSensorData.getOpticalFlowQoS();

		 if ((StellwertRight < 70) || (StellwertRight > 100))
		 Abweichung_SummeRight += AbweichungRight;

		 StellwertRight = KpRight * AbweichungRight;
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
		cout<< "Speed : "<<wm->rawSensorData.getOwnVelocityMotion()->translation<<endl;
//		cout << " QualityOfService : " << wm->rawSensorData.getOpticalFlowQoS() << endl;
		cout << "feedForwardLeft : " << feedForwardLeft << endl;
		cout<< "KpLeft : "<<KpLeft<<endl;
		cout<<"leftMotor : "<<left<<endl;

		cout << "feedForwardRight :" << feedForwardRight << endl;
		cout<< "KpRight : "<<KpRight<<endl<<endl;
		cout << "rightMotor : " << right << endl;

//	cout<<"leftMotor : "<<left<<"   rightStellwert: "<<StellwertRight<<endl;

		left = KpLeft;
		right = KpRight;
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
