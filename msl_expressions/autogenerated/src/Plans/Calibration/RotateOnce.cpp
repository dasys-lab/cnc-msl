#include <msl_msgs/MotionInfo.h>
#include <msl_msgs/PositionInfo.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

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
    /*PROTECTED REGION END*/
    RotateOnce::RotateOnce() :
			DomainBehaviour("RotateOnce"), precisionBuffer(PRECISION_BUFFER_SIZE)
    {
        /*PROTECTED REGION ID(con1467397900274) ENABLED START*/ //Add additional options here
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
        msl_actuator_msgs::MotionControl mc;
        rotationSpeed = getLimitedRotationSpeed(rotationSpeed + ACCELERATION); // accelerate in each iteration until max rotation speed is reached
        mc.motion.rotation = rotationSpeed;
        send(mc);

        int currentSegment = getCurrentRotationSegment();
        double currentBearing = wm->rawSensorData->getAverageBearing();
//        cout << "segment " << currentSegment << ", bearing " << currentBearing << "/" << initialBearing << ", rotspeed: " << rotationSpeed << endl;
        visitedSegments[currentSegment] = true;

        if(visitedSegments[0] && visitedSegments[1] && visitedSegments[2])
        {
    		double circDiff = circularDiff(currentBearing, initialBearing);
			precisionBuffer.add(make_shared<double>(circDiff));

			rotationSpeed = getLimitedRotationSpeed(-circDiff);

        	if(precisionBuffer.getSize() == PRECISION_BUFFER_SIZE) {
        		double diffSum = 0;
        		for(int i = 0; i < PRECISION_BUFFER_SIZE; i++) {
        			diffSum += precisionBuffer.getActualElement(i);
        		}
        		
        		if(diffSum >= 0) {
        			double endAngle = wm->rawOdometry->position.angle;
					cout << "end angle: " << endAngle << " => " ;
					lastRotationCalibError = circularDiff(initialAngle, endAngle);

					logCalibrationResult(lastRotationCalibError);

					if (lastRotationCalibError < -CALIB_ERROR_THRESHOLD)
					{
						// rotated too far => increase robot radius
						cout << "hoch" << endl;
						wm->adjustRobotRadius(0.1);
					}
					else if (lastRotationCalibError > CALIB_ERROR_THRESHOLD)
					{
						// rotated not far enough => decrease robot radius
						cout <<  "runter" << endl;
						wm->adjustRobotRadius(-0.1);
					}

					if (abs(lastRotationCalibError) < CALIB_ERROR_THRESHOLD)
					{
						cout << endl << "success" << endl;
						this->setSuccess(true);
						return;
					}
					else
					{
						// start new rotation
						cout << endl << "failure" << endl;
//						this->setSuccess(false);
						this->setFailure(true);
					}
        		} else {
        			// cout << "diffSum=" << diffSum << ", muss noch" << endl;
        		}
        	}
        }
        /*PROTECTED REGION END*/
    }
    void RotateOnce::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1467397900274) ENABLED START*/ //Add additional options here
    	rotationSpeed = ACCELERATION;
        initialBearing = wm->rawSensorData->getAverageBearing();
        precisionBuffer.clear(true);
        initialAngle = wm->rawOdometry->position.angle;
        //robotRadius = wm->
        segments[0] = initialBearing;
        segments[1] = fmod(segments[0] + 2.0 / 3 * M_PI + M_PI, (2 * M_PI)) - M_PI;
        segments[2] = fmod(segments[0] + 4.0 / 3 * M_PI + M_PI, (2 * M_PI)) - M_PI;
        visitedSegments[2] = visitedSegments[1] = visitedSegments[0] = false;
        cout << "starting angle: " << initialAngle;

        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1467397900274) ENABLED START*/ //Add additional methods here
    int RotateOnce::getCurrentRotationSegment()
    {
        double currentBearing = wm->rawSensorData->getAverageBearing();
        double minDiff = 1337; // anything greater than 2 pi actually
        int segment = -1;
        for (int i = 0; i < 3; i++)
        {
            double diff = abs(circularDiff(segments[i], currentBearing));
            if (diff < minDiff)
            {
                segment = i;
                minDiff = diff;
            }
        }
        return segment;
    }

    double RotateOnce::circularDiff(double a, double b)
    {
        double diff = a - b;
        double sign = diff / abs(diff);
        double absDiff = abs(diff);
        double atMost180 = min(2 * M_PI - absDiff, absDiff);
        return atMost180 * sign;
    }

    double RotateOnce::getLimitedRotationSpeed(double desiredSpeed) {
		return min(MAX_ROTATION_SPEED, max(-MAX_ROTATION_SPEED, desiredSpeed));
	}
	void RotateOnce::logCalibrationResult(double calibError)
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		supplementary::Configuration *motion = (*sc)["Motion"];

		double currentRadius = motion->get<double>("Motion", "MotionControl", "RobotRadius", NULL);

		ostringstream stream;
		ofstream os(supplementary::FileSystem::combinePaths(sc->getLogPath(), "RotationCalibration.log"),ios_base::out | ios_base::app);
		std::time_t result = std::time(nullptr);
		os << currentRadius << ";" << calibError << ";" << ctime(&result) << endl;
	}
/*PROTECTED REGION END*/
} /* namespace alica */
