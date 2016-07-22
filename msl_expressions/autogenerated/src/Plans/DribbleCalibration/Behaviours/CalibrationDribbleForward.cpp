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
            MotionControl mc = dcc.getBall();
            send(mc);
        }
        writeConfigParameters();
        /*PROTECTED REGION END*/
    }
    void CalibrationDribbleForward::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1469116853584) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1469116853584) ENABLED START*/ //Add additional methods here
    void CalibrationDribbleForward::readConfigParameters()
    {
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

        shared_ptr < vector<string> > speedsSections = (*sc)["Actuation"]->getSections("ForwardDribbleSpeeds", NULL);
        vector<double> robotSpeed2(speedsSections->size());
        vector<double> actuatorSpeed2(speedsSections->size());
        int i = 0;
        for (string subsection : *speedsSections)
        {
            robotSpeed2[i] = (*sc)["Actuation"]->get<double>("ForwardDribbleSpeeds", subsection.c_str(), "robotSpeed",
                                                             NULL);
            actuatorSpeed2[i] = (*sc)["Actuation"]->get<double>("ForwardDribbleSpeeds", subsection.c_str(),
                                                                "actuatorSpeed", NULL);
            cout << "RobotSpeed: " << robotSpeed2[i] << "actuatorSpeed: " << actuatorSpeed2[i] << endl;
            i++;
        }

        this->robotSpeed = robotSpeed2;
        this->actuatorSpeed = actuatorSpeed2;

    }

    void CalibrationDribbleForward::writeConfigParameters()
    {
    	// as an example -> improve later!
    	subsection s1;
    	s1.section = "P9";
        s1.actuatorSpeed = 300;
        s1.robotSpeed = 400;
        shared_ptr<vector<subsection>> sections;
        sections->push_back(s1);

        dcc.writeConfigParameters(sections);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
