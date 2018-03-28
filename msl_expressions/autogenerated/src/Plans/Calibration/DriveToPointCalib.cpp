using namespace std;
#include "Plans/Calibration/DriveToPointCalib.h"

/*PROTECTED REGION ID(inccpp1474278265440) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <Logger.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1474278265440) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DriveToPointCalib::DriveToPointCalib() :
            DomainBehaviour("DriveToPointCalib")
    {
        /*PROTECTED REGION ID(con1474278265440) ENABLED START*/ //Add additional options here
        query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    DriveToPointCalib::~DriveToPointCalib()
    {
        /*PROTECTED REGION ID(dcon1474278265440) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DriveToPointCalib::run(void* msg)
    {
        /*PROTECTED REGION ID(run1474278265440) ENABLED START*/ //Add additional options here
        auto me = wm->rawSensorData->getOwnPositionVision();
        auto ballPos = wm->ball->getEgoBallPosition();
        if (!me.operator bool())
        {
            return;
        }
        auto egoTarget = alloTarget.alloToEgo(*me);

        msl_actuator_msgs::MotionControl mc;

        /*if (ballPos != nullptr)
         {
         mc = RobotMovement::moveToPointCarefully(egoTarget, ballPos, 0);
         }
         else
         {*/
        // replaced with new moveToPoint method
        //mc = RobotMovement::moveToPointCarefully(egoTarget, make_shared < geometry::CNPoint2D > (-1000.0, 0.0), 0);
        query->egoDestinationPoint = egoTarget;
        query->egoAlignPoint = make_shared < geometry::CNPoint2D > (-1000.0, 0.0);
        mc = this->robot->robotMovement->moveToPoint(query);
        mc.motion.translation = 500;
        //}

        if (egoTarget->length() < 100)
        {
            mc.motion.translation = 0;
            send(mc);
            sleep(1);
            //cout << "DriveToPoint: Success" << endl;
            this->logger->log(this->getName(), "success", msl::LogLevels::debug);

            this->setSuccess(true);
        }

        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        /*PROTECTED REGION END*/
    }
    void DriveToPointCalib::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1474278265440) ENABLED START*/ //Add additional options here
        string tmp;
        bool success = true;
        success &= getParameter("X", tmp);
        try
        {
            if (success)
            {
                alloTarget.x = stod(tmp);
            }
            success &= getParameter("Y", tmp);
            if (success)
            {
                alloTarget.y = stod(tmp);
            }
        }
        catch (exception& e)
        {
            //cerr << "Could not cast the parameter properly" << endl;
            this->logger->log(this->getName(), "Could not cast parameter properly", msl::LogLevels::error);
        }
        if (!success)
        {
            //cerr << "D2P: Parameter does not exist" << endl;
        	this->logger->log(this->getName(), "Parameter does not exist", msl::LogLevels::error);
        }
        defaultTranslation = (*this->sc)["Drive"]->get<double>("Drive", "Default", "Velocity", NULL);
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1474278265440) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
