/*
 * actuator.h
 *
 *  Created on: Apr 7, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_ACTUATOR_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_ACTUATOR_H_

// ROS
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "msl_actuator_msgs/BallCatchCmd.h"
#include "msl_actuator_msgs/BallHandleCmd.h"
#include "msl_actuator_msgs/ShovelSelectCmd.h"
#include "msl_actuator_msgs/HaveBallInfo.h"
#include "msl_actuator_msgs/VisionRelocTrigger.h"
#include "msl_actuator_msgs/MotionLight.h"

// BlackLibs
#include "BlackADC.h"
#include "BlackGPIO.h"
#include "BlackI2C.h"
#include "BlackPWM.h"
#include "BlackSPI.h"

// Threads
#include <thread>
#include <mutex>
#include <condition_variable>

//eigene
#include "config.h"
#include "ballhandle.h"

using namespace BlackLib;



BallHandle		BH_right(P8_13, GPIO_67, GPIO_66, GPIO_69, GPIO_68);
BallHandle		BH_left(P8_19, GPIO_44, GPIO_45, GPIO_47, GPIO_46);

BlackADC		ADC_Light(AIN1);
uint16_t		lightbarrier;

BlackPWM		ShovelSelect(P9_14);
Shovel			shovel;

timeval			time_now;
timeval			last_ping;

ros::NodeHandle node;
//ros::Subscriber bhcSub = node.subscribe<msl_actuator_msgs::BallHandleCmd>("BallHandleControl", 25, handleBallHandleControl, this);
//ros::Subscriber sscSub = node.subscribe<msl_actuator_msgs::ShovelSelectCmd>("ShovelSelectControl", 25, handleShovelSelectControl, this);
//ros::Subscriber mlcSub = node.subscribe<msl_actuator_msgs::MotionLight>("CNActuator/MotionLight", 25, handleMotionLight, this);
//ros::Subscriber bhcSub = node.subscribe<msl_actuator_msgs::BallHandleCmd>("BallHandleControl", 25, handleBallHandleControl);

ros::Publisher bsPub = node.advertise<msl_actuator_msgs::VisionRelocTrigger>("CNActuator/BundleStatus", 10);
ros::Publisher brtPub = node.advertise<std_msgs::Empty>("CNActuator/BundleRestartTrigger", 10);
ros::Publisher vrtPub = node.advertise<msl_actuator_msgs::VisionRelocTrigger>("CNActuator/VisionRelocTrigger", 10);
// ros::Publisher mbcPub = node.advertise<msl_actuator_msgs::MotionBurst>("CNActuator/MotionBurst", 10);
ros::Publisher hbiPub = node.advertise<msl_actuator_msgs::HaveBallInfo>("HaveBallInfo", 10);
// ros::Publisher imuPub = node.advertise<YYeigene msg bauenYY>("IMU", 10);



// PINS

BlackGPIO i_magnet(GPIO_26, input, FastMode);		// P8 14
BlackGPIO i_accel(GPIO_27, input, FastMode);		// P8 17
BlackGPIO i_temp(GPIO_65, input, FastMode);			// P8 18
BlackGPIO i_gyro(GPIO_61, input, FastMode);			// P8 26

BlackGPIO LED_Vision(GPIO_48, output, FastMode);	// P9 15
BlackGPIO LED_Bundle(GPIO_49, output, FastMode);	// P9 23
BlackGPIO LED_Power(GPIO_20, output, FastMode);		// P9 41		// Oder GPIO_116

BlackGPIO SW_Vision(GPIO_30, input, FastMode);		// P8 07
BlackGPIO SW_Bundle(GPIO_31, input, FastMode);		// P8 07
BlackGPIO SW_Power(GPIO_31, input, FastMode);		// P8 07

BlackI2C myI2C(I2C_2, 0x22);
BlackSPI mySpi(SPI0_0, 8, SpiDefault, 200000);




#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_ACTUATOR_H_ */
