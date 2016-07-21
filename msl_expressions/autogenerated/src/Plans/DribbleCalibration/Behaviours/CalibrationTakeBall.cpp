using namespace std;
#include "Plans/DribbleCalibration/Behaviours/CalibrationTakeBall.h"

/*PROTECTED REGION ID(inccpp1469109429392) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include "container/CNPoint2D.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1469109429392) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CalibrationTakeBall::CalibrationTakeBall() :
            DomainBehaviour("CalibrationTakeBall")
    {
        /*PROTECTED REGION ID(con1469109429392) ENABLED START*/ //Add additional options here
        query = make_shared<msl::MovementQuery>();
        readConfigParameters();
        /*PROTECTED REGION END*/
    }
    CalibrationTakeBall::~CalibrationTakeBall()
    {
        /*PROTECTED REGION ID(dcon1469109429392) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void CalibrationTakeBall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1469109429392) ENABLED START*/ //Add additional options here
        readConfigParameters();

        if (wm->ball->haveBall())
        {
            // let ball continuously rotate with speedNoBall (should be by 4000)
            // read optical flow value
            // decide if ball is rotating straight
            // if ball isn't rotating straight -> correct values in config values of dribbleFactor left and right

            // if the ball is rotating straight
            // check minRotation and slowTranslationWheelSpeed -> ball need to be in kicker but he may not rotating
        }
        else
        {
            getBall();
        }
        writeConfigParameters();
        /*PROTECTED REGION END*/
    }
    void CalibrationTakeBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1469109429392) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1469109429392) ENABLED START*/ //Add additional methods here
    void CalibrationTakeBall::getBall()
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

    void CalibrationTakeBall::readConfigParameters()
    {
        supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
        speedNoBall = (*sys)["Actuation"]->get<double>("Dribble.SpeedNoBall", NULL);
        slowTranslationWheelSpeed = (*sys)["Actuation"]->get<double>("Dribble.SlowTranslationWheelSpeed", NULL);
        minRotation = (*sys)["Actuation"]->get<double>("Dribble.MinRotation", NULL);
        // left and right are swapped!!!!
        dribbleFactorLeft = (*sys)["Actuation"]->get<double>("Dribble.DribbleFactorRight", NULL);
        dribbleFactorRight = (*sys)["Actuation"]->get<double>("Dribble.DribbleFactorLeft", NULL);
    }

    void CalibrationTakeBall::writeConfigParameters()
    {
        supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (speedNoBall), "Dribble.SpeedNoBall", NULL);
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (slowTranslationWheelSpeed),
                                 "Dribble.SlowTranslationWheelSpeed", NULL);
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (minRotation), "Dribble.MinRotation", NULL);
        // left and right are swapped!!
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (dribbleFactorLeft), "Dribble.DribbleFactorRight",
                                 NULL);
        (*sys)["Actuation"]->set(boost::lexical_cast < std::string > (dribbleFactorRight), "Dribble.DribbleFactorLeft",
                                 NULL);

        (*sys)["Actuation"]->store();
    }
/*PROTECTED REGION END*/
} /* namespace alica */
