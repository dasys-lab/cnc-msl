using namespace std;
#include "Plans/Calibration/RotateOnce.h"

/*PROTECTED REGION ID(inccpp1467397900274) ENABLED START*/ //Add additional includes here
#include<MSLWorldModel.h>
#include <Game.h>
#include <RawSensorData.h>
#include <math.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1467397900274) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    RotateOnce::RotateOnce() :
            DomainBehaviour("RotateOnce")
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
    	if(initialized == false)
    	{
			uninitialzedCounter++;
    	}

    	if (initialized == false && uninitialzedCounter == 200)
		{
			wm->sendStartMotionCommand();
			return;
		}
    	else if (initialized == false && uninitialzedCounter < 200)
    	{
    		return;
    	}

    	if (initialized == false && uninitialzedCounter != 400)
		{
			return;
		}


    	if(initialized == false) {
			initialBearing = wm->rawSensorData->getAverageBearing();

			initialAngle = wm->rawOdometry->position.angle;
			//robotRadius = wm->
			segments[0] = initialBearing;
			segments[1] = fmod(segments[0] + 2.0 / 3 * M_PI + M_PI, (2 * M_PI)) - M_PI;
			segments[2] = fmod(segments[0] + 4.0 / 3 * M_PI + M_PI, (2 * M_PI)) - M_PI;
			visitedSegments[2] = visitedSegments[1] = visitedSegments[0] = false;
			cout << "starting angle: " << initialAngle;
			initialized = true;
		}
    	msl_actuator_msgs::MotionControl mc;
        mc.motion.rotation = 0.5;
        send(mc);
        int currentSegment = getCurrentRotationSegment();
        double currentBearing = wm->rawSensorData->getAverageBearing();
        cout << "segment " << currentSegment << ", bearing " << currentBearing << "/" << initialBearing << endl;
        visitedSegments[currentSegment] = true;

        if (visitedSegments[0] && visitedSegments[1] && visitedSegments[2]
                && abs(circularDiff(currentBearing, initialBearing)) < CALIB_ERROR_THRESHOLD)
        {
			double endAngle = wm->rawOdometry->position.angle;
			cout << "end angle: " << endAngle;
			lastRotationCalibError = circularDiff(initialAngle, endAngle);

            if(lastRotationCalibError < -CALIB_ERROR_THRESHOLD)
            {
            	// rotated too far => increase robot radius
            	cout << endl << "hoch" << endl << endl;
            	wm->adjustRobotRadius(1);
            	wm->sendKillMotionCommand();
            }
            else if(lastRotationCalibError > CALIB_ERROR_THRESHOLD)
            {
            	// rotated not far enough => decrease robot radius
            	cout << endl << "runter" << endl << endl;
            	wm->adjustRobotRadius(-1);
            	wm->sendKillMotionCommand();
            }

            if(abs(lastRotationCalibError) < CALIB_ERROR_THRESHOLD)
            {
            	cout << "success" << endl;
            	this->setSuccess(true);
            	return;
            }
            else
            {
            	// start new rotation
            	cout << "failure" << endl;
				this->setSuccess(false);
				this->setFailure(true);
            }

            cout << "finished run()" << endl;
        }
        /*PROTECTED REGION END*/
    }
    void RotateOnce::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1467397900274) ENABLED START*/ //Add additional options here
		uninitialzedCounter = 0;
		initialized = false;
		cout << "initialized parameters" << endl;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1467397900274) ENABLED START*/ //Add additional methods here
    int RotateOnce::getCurrentRotationSegment()
    {
        double currentBearing = wm->rawSensorData->getAverageBearing();
        double minDiff = 10; // anything greater than 2 pi actually
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
    	double atMost180 = min(2*M_PI - absDiff, absDiff);
    	return atMost180 * sign;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
