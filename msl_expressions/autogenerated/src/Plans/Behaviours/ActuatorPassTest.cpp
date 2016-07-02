using namespace std;
#include "Plans/Behaviours/ActuatorPassTest.h"

/*PROTECTED REGION ID(inccpp1467309160739) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <MSLWorldModel.h>
#include <Ball.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1467309160739) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    ActuatorPassTest::ActuatorPassTest() :
            DomainBehaviour("ActuatorPassTest")
    {
        /*PROTECTED REGION ID(con1467309160739) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    ActuatorPassTest::~ActuatorPassTest()
    {
        /*PROTECTED REGION ID(dcon1467309160739) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void ActuatorPassTest::run(void* msg)
    {
        /*PROTECTED REGION ID(run1467309160739) ENABLED START*/ //Add additional options here
//		auto egoBallPos = wm->ball->getEgoBallPosition();
        shared_ptr < geometry::CNPoint2D > egoBallPos = this->wm->ball->getEgoBallPosition();
        msl_actuator_msgs::MotionControl mc;
        mc.motion.rotation = egoBallPos->rotate(M_PI)->angleTo() * 4;

        int tolerance = 20;
        if (egoBallPos->y < 0 + tolerance)
        {
            mc.motion.angle = M_PI / 2;
            mc.motion.translation = 300;
        }
        else if (egoBallPos->y >= 0 - tolerance)
        {
            mc.motion.angle = 3 * M_PI / 2;
            mc.motion.translation = 300;
        }
        else
        {
        }
            mc.motion.translation = 0;

        msl_actuator_msgs::BallHandleCmd bhc;
        bhc.leftMotor = (*sys)["Actuation"]->get<double>("Dribble.SpeedNoBall", NULL);
        bhc.leftMotor = (*sys)["Actuation"]->get<double>("Dribble.SpeedNoBall", NULL);
        send(mc);
        send(bhc);
        /*PROTECTED REGION END*/
    }
    void ActuatorPassTest::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1467309160739) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1467309160739) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
