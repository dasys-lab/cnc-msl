#include "Plans/Penalty/DriveToPenaltyStart.h"

/*PROTECTED REGION ID(inccpp1459609457478) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <nonstd/optional.hpp>

using std::cout;
using std::endl;
using geometry::CNPointAllo;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1459609457478) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DriveToPenaltyStart::DriveToPenaltyStart() :
            DomainBehaviour("DriveToPenaltyStart")
    {
        /*PROTECTED REGION ID(con1459609457478) ENABLED START*/ //Add additional options here
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
        msl::RobotMovement rm;
        auto me = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        if (!me.has_value())
        {
            return;
        }
        auto egoTarget = CNPointAllo(0.0, 0.0).toEgo(*me);
        auto egoAlignPoint = wm->field->posOppGoalMid().toEgo(*me);

        msl_actuator_msgs::MotionControl mc;

        // repalced with new moveToPoint method
//        mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoAlignPoint, 0);
        query.egoDestinationPoint = egoTarget;
        query.egoAlignPoint = egoAlignPoint;

        mc = rm.moveToPoint(query);

        if (egoTarget.length() < 250)
        {
            this->setSuccess(true);
        }

        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        else
        {
            cout << "Motion command is NaN!" << endl;
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
