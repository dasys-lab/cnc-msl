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
        mc.motion.rotation = rotationSpeed;
        send(mc);
        int currentSegment = getCurrentRotationSegment();
        double currentBearing = wm->rawSensorData->getAverageBearing();
        cout << "segment " << currentSegment << ", bearing " << currentBearing << "/" << initialBearing << ", rotspeed: " << rotationSpeed << endl;
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
        			cout << "diffSum=" << diffSum << ", hier könnten wir aufhören" << endl;

					double endAngle = wm->rawOdometry->position.angle;
					cout << "end angle: " << endAngle;
					lastRotationCalibError = circularDiff(initialAngle, endAngle);

					if (lastRotationCalibError < -CALIB_ERROR_THRESHOLD)
					{
						// rotated too far => increase robot radius
						cout << endl << "hoch" << endl << endl;
						wm->adjustRobotRadius(1);
					}
					else if (lastRotationCalibError > CALIB_ERROR_THRESHOLD)
					{
						// rotated not far enough => decrease robot radius
						cout << endl << "runter" << endl << endl;
						wm->adjustRobotRadius(-1);
					}

					if (abs(lastRotationCalibError) < CALIB_ERROR_THRESHOLD)
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
    	rotationSpeed = MAX_ROTATION_SPEED;
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
        double atMost180 = min(2 * M_PI - absDiff, absDiff);
        return atMost180 * sign;
    }

    double RotateOnce::getLimitedRotationSpeed(double desiredSpeed) {
		return min(MAX_ROTATION_SPEED, max(-MAX_ROTATION_SPEED, desiredSpeed));
    }
/*PROTECTED REGION END*/
} /* namespace alica */
