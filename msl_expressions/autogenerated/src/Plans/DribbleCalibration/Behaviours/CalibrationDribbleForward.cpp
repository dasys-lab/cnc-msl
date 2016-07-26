using namespace std;
#include "Plans/DribbleCalibration/Behaviours/CalibrationDribbleForward.h"

/*PROTECTED REGION ID(inccpp1469116853584) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include "container/CNPoint2D.h"

#define MAX_SPEED 4000
#define SECTIONS_SIZE 10

#define DEBUG
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1469116853584) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CalibrationDribbleForward::CalibrationDribbleForward() :
            DomainBehaviour("CalibrationDribbleForward")
    {
        /*PROTECTED REGION ID(con1469116853584) ENABLED START*/ //Add additional options here
        sections.reserve(SECTIONS_SIZE);
#ifdef DEBUG
        cout << "CalibrationDribbleForward: sections.size() = " << sections.size() << endl;
#endif

        haveBallCount = 0;
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
        MotionControl mc;

        if (wm->rawSensorData->getLightBarrier())
        {
            haveBallCount++;
            // if ball is in kicker
            // drive forward start slowly
            // start with 400 speed
            for (int i = 0; i < SECTIONS_SIZE; i++)
            {
                int translation = 400;
                dcc.move(DRIBBLEFORWARD, translation);
            }
            // use optical flow and light barrier data to analyze the the ball movement
            // adapt actuatorSpeed by specific robotSpeed
            // if robot can handle ball at this speed -> increase the speed and repeat
        }
        else
        {
            haveBallCount = 0;
            mc = dcc.getBall();
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
    void CalibrationDribbleForward::fillSections(shared_ptr<vector<string> > speedsSections)
    {
        int i = 0;
        for (string subsection : *speedsSections)
        {
            sections[i].name = subsection.c_str();
            sections[i].robotSpeed = (*sc)["Actuation"]->get<double>("ForwardDribbleSpeeds", subsection.c_str(),
                                                                     "robotSpeed", NULL);
            sections[i].actuatorSpeed = (*sc)["Actuation"]->get<double>("ForwardDribbleSpeeds", subsection.c_str(),
                                                                        "actuatorSpeed", NULL);
            cout << "RobotSpeed: " << sections[i].robotSpeed << "actuatorSpeed: " << sections[i].actuatorSpeed << endl;
            i++;
        }

    }

    void CalibrationDribbleForward::createSections()
    {
        double speedFactor = MAX_SPEED / SECTIONS_SIZE;
        for (int i = 0; i < SECTIONS_SIZE; i++)
        {
            sections[i].name = "P" + std::to_string(i + 1);
            sections[i].robotSpeed = i * speedFactor;
            sections[i].actuatorSpeed = i * speedFactor;
        }
    }

    void CalibrationDribbleForward::readConfigParameters()
    {
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

        shared_ptr < vector<string> > speedsSections = (*sc)["Actuation"]->getSections("ForwardDribbleSpeeds", NULL);

        if (speedsSections->size() == SECTIONS_SIZE)
        {
            fillSections (speedsSections);
        }
        else
        {
            createSections();
        }

    }

    void CalibrationDribbleForward::writeConfigParameters()
    {
        // as an example -> improve later!
//		subsection s1;
//		s1.name = "P9";
//		s1.actuatorSpeed = 300;
//		s1.robotSpeed = 400;
//		shared_ptr<vector<subsection>> sections;
//		sections->push_back(s1);

        dcc.writeConfigParameters(sections);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
