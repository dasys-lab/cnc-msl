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
		double x, y;

		if (rodo == nullptr)
		{
			cout << "Actuate RODO is empty help" << endl;
			return;
		}

		double summe = 0.0;
		static double olddistance = 0.0;

		const double Kp = 1.0;
		const double Ki = 1.0;
		const double Kd = 1.0;
		const double Sollwert=90;
		double qualityOfService = wm->rawSensorData.getOpticalFlowQoS();




		double Abweichung=0.0;
		double Abweichung_Summe=0.0;
		double Abweichung_Alt=0.0;
		double Stellwert=0.0;



		Abweichung = Sollwert - wm->rawSensorData.getOpticalFlowQoS();

			if ((Stellwert < 70)||(Stellwert > 120))
				Abweichung_Summe += Abweichung;

			Stellwert  = Kp*Abweichung;
			Stellwert += Ki*Abweichung_Summe;
			Stellwert += Kd*(Abweichung - Abweichung_Alt);

		    Abweichung_Alt = Abweichung;

		    Stellwert /= 4;
/*
			if (Stellwert > 100)
				Stellwert = 100;
			if (Stellwert < 80)
				Stellwert = 80;

			return Stellwert;

	*/





		cout << "QualityOfService WM : " << wm->rawSensorData.getOpticalFlowQoS()<< endl;
		cout << "Stellwert : " << Stellwert << endl;

		// left =  rodo->motion.translation * (1.0 / 40.0) ;
		//right = rodo->motion.translation * (1.0 / 40.0) ;

		left=-Stellwert;
		right=-Stellwert;

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
