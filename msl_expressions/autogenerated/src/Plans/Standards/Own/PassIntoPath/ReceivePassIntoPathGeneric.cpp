using namespace std;
#include "Plans/Standards/Own/PassIntoPath/ReceivePassIntoPathGeneric.h"

/*PROTECTED REGION ID(inccpp1457531583460) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "SolverType.h"
#include <Ball.h>
#include <RawSensorData.h>
#include <MSLWorldModel.h>
using geometry::CNPointAllo;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1457531583460) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    ReceivePassIntoPathGeneric::ReceivePassIntoPathGeneric() :
            DomainBehaviour("ReceivePassIntoPathGeneric")
    {
        /*PROTECTED REGION ID(con1457531583460) ENABLED START*/ //Add additional options here
        query = make_shared < Query > (wm->getEngine());

        supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
        maxVel = (*sys)["Behaviour"]->get<double>("Behaviour.MaxSpeed", NULL);
        /*PROTECTED REGION END*/
    }
    ReceivePassIntoPathGeneric::~ReceivePassIntoPathGeneric()
    {
        /*PROTECTED REGION ID(dcon1457531583460) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void ReceivePassIntoPathGeneric::run(void* msg)
    {
        /*PROTECTED REGION ID(run1457531583460) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;
        msl_actuator_msgs::MotionControl mc;
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto ballPos = wm->ball->getPositionEgo();
        if (!ownPos || !ballPos )
            return;

        bool ret = query->getSolution(SolverType::GRADIENTSOLVER, runningPlan, result);
        auto passGoal = CNPointAllo (result[0], result[1]);

        auto passBallVec = passGoal - ballPos->toAllo(*ownPos);
        //Place Robot 75cm left/right and 50cm before passpoint
        //Check for obstacles (shouldnt be there as opponents are not allowed to be here)
        if (sign > 0 && passGoal.y > 0)
        {
            sign = -1.0;
        }
        else if (sign < 0 && passGoal.y < 0)
        {
            sign = 1.0;
        }
        auto p = passGoal + passBallVec.rotateZ(sign * M_PI / 2.0).normalize() * 900;
        p = p - passBallVec.normalize() * 500;

        if (result.size() > 0)
        {
            auto driveTo = p.toEgo(*ownPos);
            // replaced with new moveToPoint method
//            mc = msl::RobotMovement::placeRobotCareBall(driveTo, passGoal->alloToEgo(*ownPos), maxVel);
            movQuery.egoDestinationPoint = driveTo;
            movQuery.egoAlignPoint = ballPos;
            mc = rm.moveToPoint(movQuery);
            if (driveTo.length() < 100)
            {
                mc.motion.translation = 0;
            }
        }
        else
        {
            return;
        }
        send(mc);
        /*PROTECTED REGION END*/
    }
    void ReceivePassIntoPathGeneric::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1457531583460) ENABLED START*/ //Add additional options here
        query->clearStaticVariables();
        query->addStaticVariable(getVariablesByName("X"));
        query->addStaticVariable(getVariablesByName("Y"));
        result.clear();
        sign = 1.0;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1457531583460) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
