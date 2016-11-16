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
            DomainBehaviour("RotateOnce")
    {
        /*PROTECTED REGION ID(con1467397900274) ENABLED START*/ //Add additional options here
        initialRadius = wm->getRobotRadius();
        radiusOffset = (NUMBER_OF_STEPS - 1) / 2 * STEP_SIZE;
        minRadius = initialRadius - radiusOffset;
        maxRadius = initialRadius + radiusOffset;
        hasInitialConfigurationBeenSet = false;
        precisionBuffer = new msl::RingBuffer<double>(PRECISION_BUFFER_SIZE);
        /*PROTECTED REGION END*/
    }
    RotateOnce::~RotateOnce()
    {
        /*PROTECTED REGION ID(dcon1467397900274) ENABLED START*/ //Add additional options here
        delete precisionBuffer;
        /*PROTECTED REGION END*/
    }
    void RotateOnce::run(void* msg)
    {
        /*PROTECTED REGION ID(run1467397900274) ENABLED START*/ //Add additional options here
        if (!(this->isSuccess() || this->isFailure()))
        {
            if (!hasInitialConfigurationBeenSet)
            {
                cout << "Kill Motion first" << endl;
                wm->setRobotRadius(minRadius);
                hasInitialConfigurationBeenSet = true;
                this->setFailure(true);
                return;
            }

            msl_actuator_msgs::MotionControl mc;
            rotationSpeed = getLimitedRotationSpeed(rotationSpeed + ACCELERATION); // accelerate in each iteration until max rotation speed is reached
            mc.motion.rotation = rotationSpeed;
            send(mc);

	int currentSegment = getCurrentRotationSegment();
	double currentBearing = wm->rawSensorData->getAverageBearing();
	// cout << "segment status: " << visitedSegments[0] << " " << visitedSegments[1] << " " << visitedSegments[2] << " " << endl; //currentSegment << ", bearing " << currentBearing << "/" << initialBearing << ", rotspeed: " << rotationSpeed << endl;
	visitedSegments[currentSegment] = true;

            if (visitedSegments[0] && visitedSegments[1] && visitedSegments[2])
            {
                double circDiff = circularDiff(currentBearing, initialBearing);
                precisionBuffer->add(make_shared<double>(circDiff));

                rotationSpeed = getLimitedRotationSpeed(-circDiff);

                if (precisionBuffer->getSize() == PRECISION_BUFFER_SIZE)
                {
                    double diffSum = 0;
                    for (int i = 0; i < PRECISION_BUFFER_SIZE; i++)
                    {
                        diffSum += precisionBuffer->getActualElement(i);
                    }

                    if (diffSum >= 0)
                    {
                        double endAngle = wm->rawOdometry->position.angle;
                        cout << "end angle: " << endAngle << " => ";
                        lastRotationCalibError = circularDiff(initialAngle, endAngle);
                        logCalibrationResult(wm->getRobotRadius(), lastRotationCalibError);
                        wm->adjustRobotRadius(STEP_SIZE);

                        if (maxRadius <= wm->getRobotRadius())
                        {
                            cout << endl << "success" << endl;
                            this->setSuccess(true);
                            return;
                        }
                        else
                        {
                            this->setFailure(true);
                        }
                    }
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
        precisionBuffer->clear(true);
        initialAngle = wm->rawOdometry->position.angle;

	// TODO this needs an explaining comment
        segments[0] = fmod(initialBearing + 1.0 / 3 * M_PI + M_PI, (2 * M_PI)) - M_PI;
        segments[1] = fmod(segments[0] + 2.0 / 3 * M_PI + M_PI, (2 * M_PI)) - M_PI;
        segments[2] = fmod(segments[0] + 4.0 / 3 * M_PI + M_PI, (2 * M_PI)) - M_PI;
        visitedSegments[0] = false;
	visitedSegments[1] = false;
	visitedSegments[2] = false;
        cout << "starting angle: " << initialAngle << ", ";

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
    }
/*PROTECTED REGION END*/
} /* namespace alica */
