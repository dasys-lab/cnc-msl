using namespace std;
#include "Plans/DribbleCalibration/Behaviours/CalibrationDribbleForward.h"

/*PROTECTED REGION ID(inccpp1469116853584) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include "container/CNPoint2D.h"
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1469116853584) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	CalibrationDribbleForward::CalibrationDribbleForward() :
			DomainBehaviour("CalibrationDribbleForward")
	{
		/*PROTECTED REGION ID(con1469116853584) ENABLED START*/ //Add additional options here
		readConfigParameters();
		/*PROTECTED REGION END*/
	}
	CalibrationDribbleForward::~CalibrationDribbleForward()
	{
		/*PROTECTED REGION ID(dcon1469116853584) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void CalibrationDribbleForward::run(void* msg)
	{
		/*PROTECTED REGION ID(run1469116853584) ENABLED START*/ //Add additional options here
		if (wm->ball->haveBall())
		{
			// if ball is in kicker
			// drive forward start slowly
			// start with 300 speed
			// use optical flow and light barrier data to analyze the the ball movement
			// adapt actuatorSpeed by specific robotSpeed
			// if robot can handle ball at this speed -> increase the speed and repeat
		}
		else
		{
			getBall();
		}
		/*PROTECTED REGION END*/
	}
	void CalibrationDribbleForward::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1469116853584) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1469116853584) ENABLED START*/ //Add additional methods here
	void CalibrationDribbleForward::getBall()
	{
		msl::RobotMovement rm;
		MotionControl mc;
		auto me = wm->rawSensorData->getOwnPositionVision();
		auto egoBallPos = wm->ball->getAlloBallPosition()->alloToEgo(*me);

		query->egoDestinationPoint = egoBallPos;
		query->egoAlignPoint = egoBallPos;

		mc = rm.moveToPoint(query);
		send(mc);
	}

	void CalibrationDribbleForward::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		shared_ptr<vector<string> > speedsSections = (*sc)["Actuation"]->getSections("ForwardDribbleSpeeds", NULL);
		vector<double> robotSpeed(speedsSections->size());
		vector<double> actuatorSpeed(speedsSections->size());
		int i = 0;
		for (string subsection : *speedsSections)
		{
			robotSpeed[i] = (*sc)["Actuation"]->get<double>("ForwardDribbleSpeeds", subsection.c_str(), "robotSpeed",
			NULL);
			actuatorSpeed[i] = (*sc)["Actuation"]->get<double>("ForwardDribbleSpeeds", subsection.c_str(),
																"actuatorSpeed", NULL);
			cout << "RobotSpeed: " << robotSpeed[i] << "actuatorSpeed: " << actuatorSpeed[i] << endl;
			i++;
		}

	}

	void CalibrationDribbleForward::writeConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		// create speedsSections

	}
/*PROTECTED REGION END*/
} /* namespace alica */
