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
    	msl_actuator_msgs::MotionControl mc;
    	mc.motion.rotation = 0.1;
		send(mc);
		int currentSegment = getCurrentRotationSegment();
		double currentBearing = wm->rawSensorData->getAverageBearing();
		cout << "segment " << currentSegment << ", bearing " << currentBearing << endl;
    	visitedSegments[currentSegment] = true;

    	if(visitedSegments[0] && visitedSegments[1] && visitedSegments[2] && circularDiff(currentBearing, initialBearing) < 0.02)
    	{
    		cout << "end angle: " << wm->rawOdometry->position.angle;
    		this->setSuccess(true);
    	}
        /*PROTECTED REGION END*/
    }
    void RotateOnce::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1467397900274) ENABLED START*/ //Add additional options here
        initialBearing = wm->rawSensorData->getAverageBearing();
        initialAngle = wm->rawOdometry->position.angle;
        segments[0] = initialBearing;
        segments[1] = fmod(segments[0] + 2.0/3 * M_PI+ M_PI, (2*M_PI)) - M_PI;
        segments[2] = fmod(segments[0] + 4.0/3 * M_PI+ M_PI, (2*M_PI)) - M_PI;
        visitedSegments[2] = visitedSegments[1] = visitedSegments[0] = false;
        cout << "starting angle: " << initialAngle;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1467397900274) ENABLED START*/ //Add additional methods here
    int RotateOnce::getCurrentRotationSegment() {
		double currentBearing = wm->rawSensorData->getAverageBearing();
		double minDiff = 10; // anything greater than 2 pi actually
		int segment = -1;
		for(int i = 0; i < 3; i++)
		{
			double diff = circularDiff(segments[i], currentBearing);
			if(diff < minDiff) {
				segment = i;
				minDiff = diff;
			}
		}
		return segment;
    }

    double RotateOnce::circularDiff(double a, double b) {
    	double absDiff = abs(a-b);
    	return min(absDiff, 2*M_PI-absDiff);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
