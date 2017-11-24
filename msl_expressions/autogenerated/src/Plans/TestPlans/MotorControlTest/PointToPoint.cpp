using namespace std;
#include "Plans/TestPlans/MotorControlTest/PointToPoint.h"

/*PROTECTED REGION ID(inccpp1489068164649) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <RawSensorData.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1489068164649) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    PointToPoint::PointToPoint() :
            DomainBehaviour("PointToPoint")
    {
        /*PROTECTED REGION ID(con1489068164649) ENABLED START*/ //Add additional options here
        this->toOwnPentalty = true;
        /*PROTECTED REGION END*/
    }
    PointToPoint::~PointToPoint()
    {
        /*PROTECTED REGION ID(dcon1489068164649) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void PointToPoint::run(void* msg)
    {
        /*PROTECTED REGION ID(run1489068164649) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;
        auto me = this->wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto ballPos = this->wm->ball->getPositionEgo();
        if (!me)
        {
            return;
        }
        if (toOwnPentalty)
        {
            this->egoTarget = this->wm->field->posOwnPenaltyMarker().toEgo(*me);
        }
        else
        {
            this->egoTarget = this->wm->field->posOppPenaltyMarker().toEgo(*me);
        }
        msl_actuator_msgs::MotionControl mc;
        this->query.egoDestinationPoint = nonstd::make_optional<geometry::CNPointEgo>(this->egoTarget);

        mc = rm.moveToPoint(query);

        if (this->egoTarget.length() < 250)
        {
            msl_actuator_msgs::MotionControl mc2;
            mc2.motion.rotation = 1000;
            send(mc2);
            this->toOwnPentalty = !this->toOwnPentalty;
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
    void PointToPoint::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1489068164649) ENABLED START*/ //Add additional options here
        this->toOwnPentalty = true;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1489068164649) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
