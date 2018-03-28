using namespace std;
#include "Plans/Penalty/DriveToPenaltyStart.h"

/*PROTECTED REGION ID(inccpp1459609457478) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <Logger.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1459609457478) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DriveToPenaltyStart::DriveToPenaltyStart() :
            DomainBehaviour("DriveToPenaltyStart")
    {
        /*PROTECTED REGION ID(con1459609457478) ENABLED START*/ //Add additional options here
        query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    DriveToPenaltyStart::~DriveToPenaltyStart()
    {
        /*PROTECTED REGION ID(dcon1459609457478) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DriveToPenaltyStart::run(void* msg)
    {
        /*PROTECTED REGION ID(run1459609457478) ENABLED START*/ //Add additional options here
        auto me = wm->rawSensorData->getOwnPositionVision();
        auto ballPos = wm->ball->getEgoBallPosition();
        if (me == nullptr)
        {
            return;
        }
        auto egoTarget = make_shared < geometry::CNPoint2D > (0.0, 0.0)->alloToEgo(*me);
        auto egoAlignPoint = wm->field->posOppGoalMid()->alloToEgo(*me);

        msl_actuator_msgs::MotionControl mc;

        // repalced with new moveToPoint method
//        mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoAlignPoint, 0);
        query->egoDestinationPoint = egoTarget;
        query->egoAlignPoint = egoAlignPoint;

        mc = this->robot->robotMovement->moveToPoint(query);

        if (egoTarget->length() < 250)
        {
            this->setSuccess(true);
        }

        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        else
        {
//            cout << "Motion command is NaN!" << endl;
            this->logger->log(this->getName(), "Motion command is NaN!", msl::LogLevels::warn);
        }
        /*PROTECTED REGION END*/
    }
    void DriveToPenaltyStart::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1459609457478) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1459609457478) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
