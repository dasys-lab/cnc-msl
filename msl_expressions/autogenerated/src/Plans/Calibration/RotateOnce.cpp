using namespace std;
#include "Plans/Calibration/RotateOnce.h"

/*PROTECTED REGION ID(inccpp1467397900274) ENABLED START*/ //Add additional includes here
#include <MSLWorldModel.h>
#include <Game.h>
#include <RawSensorData.h>
#include <math.h>
#include <ctime>
#include <SystemConfig.h>
#include <FileSystem.h>
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1467397900274) ENABLED START*/ //initialise static variables here
	int RotateOnce::logCounter = 0;

	/*PROTECTED REGION END*/
	RotateOnce::RotateOnce() :
			DomainBehaviour("RotateOnce")
	{
		/*PROTECTED REGION ID(con1467397900274) ENABLED START*/ //Add additional options here
		robotRadius = wm->getRobotRadius();
		lastMotionBearing = 0;
		lastIMUBearing = 0;
		rotationSpeed = 0;
		/*PROTECTED REGION END*/
	}
	RotateOnce::~RotateOnce()
	{
		/*PROTECTED REGION ID(dcon1467397900274) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void RotateOnce::run(void* msg)
	{
		/*PROTECTED REGION ID(run1467397900274) ENABLED START*/ //Add additional options here
		if (this->isSuccess())
		{
			return;
		}
		msl_actuator_msgs::MotionControl mc;
		rotationSpeed = getLimitedRotationSpeed(rotationSpeed + ACCELERATION); // accelerate in each iteration until max rotation speed is reached
		mc.motion.rotation = rotationSpeed;
		send(mc);

		double currentIMUBearing = getIMUBearing();
		double currentMotionBearing = getMotionBearing();
		double circDiff = circularDiff(currentIMUBearing, lastIMUBearing);
		double iR, mR;

		cout << currentIMUBearing << ";";
		iR = updateRotationCount(currentIMUBearing, lastIMUBearing, imuRotations);
		cout << ";";
		cout << currentMotionBearing  << ";";
		mR = updateRotationCount(currentMotionBearing, lastMotionBearing, motionRotations);
		cout << ";";
		cout << iR-mR << endl;

		// cout << "buffer" << endl;
//		double endAngle = wm->rawOdometry->position.angle;
//		cout << "end angle: " << endAngle << " => ";
//		lastRotationCalibError = circularDiff(lastMotionBearing, endAngle);
//		logCalibrationResult(wm->getRobotRadius(), lastRotationCalibError);
//		measurements[1]->y = lastRotationCalibError;
		// wm->adjustRobotRadius(STEP_SIZE);
//		this->setSuccess(true);
		/*PROTECTED REGION END*/
	}
	void RotateOnce::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1467397900274) ENABLED START*/ //Add additional options here
		rotationSpeed = ACCELERATION;
		lastMotionBearing = getMotionBearing();
		lastIMUBearing = getIMUBearing();
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1467397900274) ENABLED START*/ //Add additional methods here
	/**
	 * TODO needs doc
	 */
	double RotateOnce::updateRotationCount(double currentBearing, double &lastBearing, int &rotations)
	{
		double circDiff = circularDiff(currentBearing, lastBearing);
		double currentNormedBearing = (currentBearing + M_PI)/ (2*M_PI);
		double lastNormedBearing = (lastBearing + M_PI)/ (2*M_PI);
		lastBearing = currentBearing;

//		cout << "lst " << lastBearing << "; ";
//		cout << "cur " << currentBearing << "; ";
//		cout << "cD " << circDiff << "; ";
//		cout << "rots " << (rotations + currentNormedBearing);

		if (circDiff < 0 || circDiff > CIRCDIFF_THRESHOLD)
		{
			return 0;
		}

		if (lastNormedBearing > currentNormedBearing)
		{
			rotations++;
			cout << "1";
		}

		return rotations + currentNormedBearing;
	}

	double RotateOnce::getMotionBearing()
	{
		return wm->rawOdometry->position.angle;
	}

	double RotateOnce::getIMUBearing()
	{
		return wm->rawSensorData->getAverageBearing();
	}

	double RotateOnce::circularDiff(double a, double b)
	{
		double diff = a - b;
		if(abs(diff) > M_PI)
		{
			diff = 2*M_PI - diff;
			while(diff > M_PI)
			{
				diff -= 2 * M_PI;
			}
			diff *= -1;
		}

		return diff;
	}

	double RotateOnce::getLimitedRotationSpeed(double desiredSpeed)
	{
		return min(MAX_ROTATION_SPEED, max(-MAX_ROTATION_SPEED, desiredSpeed));
	}
	void RotateOnce::logCalibrationResult(double currentRadius, double calibError)
	{
		cout << currentRadius << endl;
		// TODO why don't we use the already defined sc for adjusting the robot radius?
		std::string logfilePath = supplementary::FileSystem::combinePaths(sc->getLogPath(), "RotationCalibration.log");
		ofstream os(logfilePath, ios_base::out | ios_base::app);
		std::time_t result = std::time(nullptr);
		os << currentRadius << "\t" << calibError << "\t" << ctime(&result) << endl;
		os.flush();
		os.close();

		RotateOnce::logCounter++;
	}
/*PROTECTED REGION END*/
} /* namespace alica */
