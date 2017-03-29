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
#include "ConsoleCommandHelper.h"
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
        imuRotations = 0;
        motionRotations = 0;
        rotationSpeed = 0;
        diffOffset = 0;
        diffOffsetInitialized = false;
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
        cout.precision(4);
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
        bool isFullIMURotation = false;
        bool isFullMotionRotation = false;

        iR = updateRotationCount(currentIMUBearing, lastIMUBearing, imuRotations, isFullIMURotation);
        mR = updateRotationCount(currentMotionBearing, lastMotionBearing, motionRotations, isFullMotionRotation);

        if (isFullIMURotation)
        {
            // if this is the first full IMU rotation, initialize the IMU rotation offset
            if (!diffOffsetInitialized)
            {
                diffOffset = iR - mR;
                cout << "set diffOffset to " << diffOffset << endl;
                diffOffsetInitialized = true;
            }

            double currentDiff = (iR - diffOffset) - mR;

            cout << currentIMUBearing << "\t";
            cout << "\t";
            cout << currentMotionBearing << "\t";
            cout << "\t";
            cout << iR << "\t" << mR << "\t" << currentDiff << endl;

            logIMUMotionDifference(currentDiff);

            if (iR > MAX_ROTATIONS)
            {
                // MAX_ROTATIONS reached - calibration finished!
                cout << "MAX_ROTATIONS reached - calibration finished!" << endl;
                this->setSuccess(true);
            }
            else if (iR > MIN_ROTATIONS && abs(currentDiff) > MIN_BEARING_DIFF_FOR_REGRESSION)
            {
                // MIN_BEARING_DIFF_FOR_REGRESSION reached - we can start a regression calculation in order to improve on the RobotRadius
                cout
                        << "MIN_BEARING_DIFF_FOR_REGRESSION reached - we can start a regression calculation in order to improve on"
                        << endl;
                calculateRadius();
                this->setFailure(true);
            }
        }

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
        imuRotations = 0;
        motionRotations = 0;
        diffOffset = 0;
        diffOffsetInitialized = false;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1467397900274) ENABLED START*/ //Add additional methods here
    void RotateOnce::calculateRadius()
    {
        stringstream cmd;
        string logfile = supplementary::FileSystem::combinePaths(sc->getLogPath(), "RotationCalibration.log");

        ifstream infile(logfile);
        string firstLine;
        // TODO needed?
//		std::getline(infile, firstLine);
//		std::getline(infile, firstLine);
        string fileContent;
        string lineCppString;
        while (std::getline(infile, lineCppString).eof() == false)
        {
            fileContent += lineCppString;
            fileContent += "\n";
        }
        fileContent += lineCppString;
        infile.close();
        ofstream ofstr(logfile);
        ofstr << fileContent;
        ofstr.flush();
        ofstr.close();

//		cmd << "gnuplot -persist -e \"f(x) = a*x+b; fit f(x) \\\"";
//		cmd << logfile;
//		cmd << "\\\" u 1:2 via a,b; plot \\\"";
//		cmd << logfile;
//		cmd << "\\\", f(x), 0; print sprintf(\\\"robotRadius=%f\\\",-b/a)\" 2>&1";

        cmd << "gnuplot -persist -e \"f(x) = a*x+b; fit f(x) '";
        cmd << logfile;
        cmd << "' u (column(0)):1 via a,b; plot '";
        cmd << logfile;
        cmd << "' , f(x), 0; print sprintf('slope=%f',a)\" 2>&1";

        cout << cmd.str() << endl;
        string gnuplotReturn = supplementary::ConsoleCommandHelper::exec(cmd.str().c_str());
        // cout << "gnuplotReturn: " << gnuplotReturn << endl;

        /* I like C!
         * I don't */
        // match plotted value
        const int max_line_size = 400;
        const int output_size = gnuplotReturn.size() + 1;
        const char *ret = gnuplotReturn.c_str();
        char output[output_size];
        char *line;
        char *lastline;
        char *slopeStr;

        strncpy(output, gnuplotReturn.c_str(), output_size);

        // Get last line
        line = strtok(output, "\n");
        while ((line = strtok(NULL, "\n")) != NULL)
            lastline = line;

        // Get the radius as string
        strtok(lastline, "="); // eats slope
        slopeStr = strtok(NULL, "=");
        if (slopeStr == NULL)
        {
            cerr << "ERROR parsing gnuplot output" << endl;
            this->setSuccess(true); // not really tho
            return;
        }

        // Convert it to double
        double calculatedSlope = strtod(slopeStr, NULL);
        double newRadius = robotRadius * (1 - calculatedSlope);
        cout << "old radius: " << robotRadius << endl;
        cout << "new radius: " << newRadius << endl;
        robotRadius = newRadius;
        wm->setRobotRadius(newRadius);
    }

    /**
     * TODO needs doc
     */
    double RotateOnce::updateRotationCount(double currentBearing, double &lastBearing, int &rotations,
                                           bool &isfullRotation)
    {
        double circDiff = circularDiff(currentBearing, lastBearing);
        double currentNormedBearing = (currentBearing + M_PI) / (2 * M_PI);
        double lastNormedBearing = (lastBearing + M_PI) / (2 * M_PI);
        lastBearing = currentBearing;

//		cout << "lst " << lastBearing << "; ";
//		cout << "cur " << currentBearing << "; ";
//		cout << "cD " << circDiff << "; ";
//		cout << "rots " << (rotations + currentNormedBearing);

        if (circDiff < 0 || circDiff > CIRCDIFF_THRESHOLD)
        {
            return 0;
        }

        isfullRotation = lastNormedBearing > currentNormedBearing;

        if (isfullRotation)
        {
            rotations++;
            // cout << "1";
        }

        // cout << "\t";

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
        // DEFINITELY SELF DOCUMENTING CODE
        // (maybe not)
        // TODO

        double diff = a - b;
        if (abs(diff) > M_PI)
        {
            diff = 2 * M_PI - diff;
            while (diff > M_PI)
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
    void RotateOnce::logIMUMotionDifference(double imuMotionDifference)
    {
        // TODO why don't we use the already defined sc for adjusting the robot radius?
        std::string logfilePath = supplementary::FileSystem::combinePaths(sc->getLogPath(), "RotationCalibration.log");
        ofstream os(logfilePath, ios_base::out | ios_base::app);
        os << imuMotionDifference << endl;
        os.flush();
        os.close();

        RotateOnce::logCounter++;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
