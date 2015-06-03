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

		double arithmeticAverage = 0.0;
		double newParamer = wm->rawSensorData.getOwnVelocityMotion()->translation;
		list<double>::iterator parameter;

		if (arithmeticAverageBox.size() == 4)
		{
			arithmeticAverageBox.pop_back();
		}

		arithmeticAverageBox.push_front(newParamer);

		for (parameter = arithmeticAverageBox.begin(); parameter != arithmeticAverageBox.end(); parameter++)
		{
			arithmeticAverage += *parameter;
		}

		arithmeticAverage = arithmeticAverage / 8;

		//PIDControllerLeft

		double x, lefty, righty, feedForwardLeft, feedForwardRight;
		double KvLeft, KvRight;
		double qualityOfService = wm->rawSensorData.getOpticalFlowQoS();
		x = wm->rawSensorData.getOwnVelocityMotion()->angle;

		righty = (x * x * 0.6 - x * 0.95 - 1.4);
		feedForwardRight = max(min(righty, 1.2), -1.2);



		KvRight = (feedForwardRight * arithmeticAverage * 1 / 35);

	

		//Function for Right

		lefty = (0.6 * x * x + 0.95 * x - 1.4);

		feedForwardLeft = max(min(lefty, 1.2), -1.2);
		


		KvLeft = (feedForwardLeft * arithmeticAverage * 1 / 35);

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
		cout << "Speed : " << arithmeticAverage << endl;
		cout << " QualityOfService : " << wm->rawSensorData.getOpticalFlowQoS() << endl;
		cout << "Kvright : " << KvRight << endl;
		cout << "StellwertRight : " << StellwertRight << endl;
		cout << "Kvleft : " << KvLeft << endl;
		cout << "StellwertLeft : " << StellwertLeft << endl << endl;

//	cout<<"leftMotor : "<<left<<"   rightStellwert: "<<StellwertRight<<endl;

		left = StellwertLeft;
		right = StellwertRight;
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
