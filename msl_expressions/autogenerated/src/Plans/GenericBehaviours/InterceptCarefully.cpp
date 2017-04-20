using namespace std;
#include "Plans/GenericBehaviours/InterceptCarefully.h"

/*PROTECTED REGION ID(inccpp1427703218101) ENABLED START*/ // Add additional includes here
#include <Ball.h>
#include <RawSensorData.h>
#include <cnc_geometry/CNPointAllo.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <pathplanner/PathProxy.h>
#include <pathplanner/evaluator/PathEvaluator.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1427703218101) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
InterceptCarefully::InterceptCarefully()
    : DomainBehaviour("InterceptCarefully")
{
    /*PROTECTED REGION ID(con1427703218101) ENABLED START*/ // Add additional options here
    interceptCarfullyRotateP = (*sc)["Drive"]->get<double>("Drive.Carefully.RotateP", NULL);
    defaultTranslation = (*sc)["Drive"]->get<double>("Drive.Default.Velocity", NULL);
    catchRadius = (*sc)["Drive"]->get<double>("Drive.Carefully.CatchRadius", NULL);
    /*PROTECTED REGION END*/
}
InterceptCarefully::~InterceptCarefully()
{
    /*PROTECTED REGION ID(dcon1427703218101) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void InterceptCarefully::run(void *msg)
{
    /*PROTECTED REGION ID(run1427703218101) ENABLED START*/ // Add additional options here
    auto ownPos = this->wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
    auto egoTarget = this->wm->ball->getPositionEgo();

    if (!ownPos || !egoTarget)
    {
        return;
    }

    if (auto pathPlanningResult =
            msl::PathProxy::getInstance()->getEgoDirection(*egoTarget, msl::PathEvaluator(), nullptr))
    {
        egoTarget = pathPlanningResult;
    }

    msl_actuator_msgs::MotionControl mc;
    mc.motion.angle = egoTarget->angleZ();
    mc.motion.rotation = egoTarget->rotateZ(M_PI).angleZ() * interceptCarfullyRotateP;
    if (egoTarget->length() > this->catchRadius)
    {
        mc.motion.translation = min(defaultTranslation, egoTarget->length());
    }
    else
    {
        mc.motion.translation = 0;
    }

    send(mc);
    /*PROTECTED REGION END*/
}
void InterceptCarefully::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1427703218101) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1427703218101) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
