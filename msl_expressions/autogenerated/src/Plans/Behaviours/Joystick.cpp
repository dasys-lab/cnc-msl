using namespace std;
#include "Plans/Behaviours/Joystick.h"

/*PROTECTED REGION ID(inccpp1421854975890) ENABLED START*/ //Add additional includes here
#include <msl_msgs/JoystickCommand.h>
#include <msl_actuator_msgs/KickControl.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_actuator_msgs/KickControl.h>
#include "MSLWorldModel.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1421854975890) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Joystick::Joystick() :
            DomainBehaviour("Joystick")
    {
        /*PROTECTED REGION ID(con1421854975890) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    Joystick::~Joystick()
    {
        /*PROTECTED REGION ID(dcon1421854975890) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Joystick::run(void* msg)
    {
        /*PROTECTED REGION ID(run1421854975890) ENABLED START*/ //Add additional options here
        msl_msgs::JoystickCommandPtr joy = wm->getJoystickCommandInfo();
        msl_actuator_msgs::MotionControl mc;
        msl_actuator_msgs::BallHandleCmd bhc;
        msl_actuator_msgs::KickControl kc;

        if (!joy.operator bool())
        {
            return;
        }

        switch (joy->selectedActuator)
        {
            case msl_msgs::JoystickCommand::ALL:
                mc.motion = joy->motion;
                mc.senderID = joy->robotId;

                bhc.senderID = joy->robotId;
                bhc.leftMotor = joy->ballHandleLeftMotor;
                bhc.rightMotor = joy->ballHandleRightMotor;

                kc.senderID = joy->robotId;
                kc.power = joy->kickPower;
                kc.extension = joy->shovelIdx;
                kc.extTime = 1;
                kc.forceVoltage = false;

                send(kc);
                send(bhc);
                send(mc);

                break;
            case msl_msgs::JoystickCommand::BALL_HANDLE_ONLY:
                bhc.senderID = joy->robotId;
                bhc.leftMotor = joy->ballHandleLeftMotor;
                bhc.rightMotor = joy->ballHandleRightMotor;
                send(bhc);
                break;
            case msl_msgs::JoystickCommand::KICKER_ONLY:
                //TODO: Check and eventually remove parts from kickControl msgs
                kc.senderID = joy->robotId;
                kc.power = joy->kickPower;
                kc.extension = joy->shovelIdx;
                kc.extTime = 1;
                kc.forceVoltage = false;
                send(kc);
                break;
            case msl_msgs::JoystickCommand::MOTION_ONLY:
                mc.motion = joy->motion;
                mc.senderID = joy->robotId;
                send(mc);
                break;
            case msl_msgs::JoystickCommand::NO_BALL_HANDLE:
                kc.senderID = joy->robotId;
                kc.power = joy->kickPower;
                kc.extension = joy->shovelIdx;
                kc.extTime = 1;
                kc.forceVoltage = false;

                mc.motion = joy->motion;
                mc.senderID = joy->robotId;

                send(mc);
                send(kc);
                break;
            case msl_msgs::JoystickCommand::NO_KICKER:
                mc.motion = joy->motion;
                mc.senderID = joy->robotId;

                bhc.senderID = joy->robotId;
                bhc.leftMotor = joy->ballHandleLeftMotor;
                bhc.rightMotor = joy->ballHandleRightMotor;

                send(bhc);
                send(mc);
                break;
            case msl_msgs::JoystickCommand::NO_MOTION:
                bhc.senderID = joy->robotId;
                bhc.leftMotor = joy->ballHandleLeftMotor;
                bhc.rightMotor = joy->ballHandleRightMotor;

                kc.senderID = joy->robotId;
                kc.power = joy->kickPower;
                kc.extension = joy->shovelIdx;
                kc.extTime = 1;
                kc.forceVoltage = false;

                send(kc);
                send(bhc);
                break;
            case msl_msgs::JoystickCommand::NOTHING:
                break;
        }
        /*PROTECTED REGION END*/
    }
    void Joystick::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1421854975890) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1421854975890) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
