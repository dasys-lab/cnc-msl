//using namespace std;
//#include "Plans/Behaviours/Joystick.h"
//
///*PROTECTED REGION ID(inccpp1421854975890) ENABLED START*/ //Add additional includes here
//#include <msl_msgs/JoystickCommand.h>
//#include <msl_actuator_msgs/KickControl.h>
//#include <msl_actuator_msgs/BallHandleCmd.h>
//#include <msl_actuator_msgs/MotionControl.h>
//#include <msl_actuator_msgs/KickControl.h>
//#include <msl_actuator_msgs/ShovelSelectCmd.h>
//#include "MSLWorldModel.h"
//#include <RawSensorData.h>
///*PROTECTED REGION END*/
//namespace alica
//{
//    /*PROTECTED REGION ID(staticVars1421854975890) ENABLED START*/ //initialise static variables here
//    /*PROTECTED REGION END*/
//    Joystick::Joystick() :
//            DomainBehaviour("Joystick")
//    {
//        /*PROTECTED REGION ID(con1421854975890) ENABLED START*/ //Add additional options here
//        /*PROTECTED REGION END*/
//    }
//    Joystick::~Joystick()
//    {
//        /*PROTECTED REGION ID(dcon1421854975890) ENABLED START*/ //Add additional options here
//        /*PROTECTED REGION END*/
//    }
//    void Joystick::run(void* msg)
//    {
//        /*PROTECTED REGION ID(run1421854975890) ENABLED START*/ //Add additional options here
//        auto joy = wm->rawSensorData->getJoystickCommandsBuffer().getLastValidContent();
//        if (!joy)
//        {
//            msl_actuator_msgs::MotionControl mc;
//            send(mc);
//            return;
//        }
//
////		if (lastProcessedCmd == joy) // only process new commands from WM
////		{
////			msl_actuator_msgs::MotionControl mc;
////			send(mc);
////			return;
////		}
//
//        //cout << "Joystick-Beh: " << *joy << endl;
//        if (!std::isnan(joy->motion.translation) && !std::isnan(joy->motion.rotation) && !std::isnan(joy->motion.angle))
//        {
//            msl_actuator_msgs::MotionControl mc;
//            mc.motion = joy->motion;
//            send(mc);
//        }
//        else
//        {
//            msl_actuator_msgs::MotionControl mc;
//            send(mc);
//            //cout << "Joystick: Some Motion Value is NaN!" << endl;
//        }
//
//        if (joy->ballHandleState == msl_msgs::JoystickCommand::BALL_HANDLE_ON)
//        {
//            //cout << "Joy Ball handle on" << endl;
//            msl_actuator_msgs::BallHandleCmd bhc;
//            bhc.leftMotor = joy->ballHandleLeftMotor;
//            bhc.rightMotor = joy->ballHandleRightMotor;
//            send(bhc);
//        }
//
//        msl_actuator_msgs::ShovelSelectCmd ssc;
//
//        if (joy->shovelIdx == 0)
//        {
//            ssc.passing = true;
//        }
//        else
//        {
//            ssc.passing = false;
//        }
//        send(ssc);
//
//        if (joy->kick == true)
//        {
//            msl_actuator_msgs::KickControl kc;
//            kc.power = joy->kickPower;
//            kc.extension = joy->shovelIdx;
//            kc.extTime = 1;
//            kc.forceVoltage = false;
//            send(kc);
//        }
//
//        lastProcessedCmd = joy;
//
//        /*PROTECTED REGION END*/
//    }
//    void Joystick::initialiseParameters()
//    {
//        /*PROTECTED REGION ID(initialiseParameters1421854975890) ENABLED START*/ //Add additional options here
//        /*PROTECTED REGION END*/
//    }
///*PROTECTED REGION ID(methods1421854975890) ENABLED START*/ //Add additional methods here
///*PROTECTED REGION END*/
//} /* namespace alica */
