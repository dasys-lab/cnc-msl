/*
 * config.h
 *
 *  Created on: Mar 12, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_CONFIG_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_CONFIG_H_

#include <iostream>
#include <signal.h>
#include <sstream>
#include <stdint.h>
#include <sys/time.h>

// Threads
#include <thread>
#include <mutex>
#include <condition_variable>

#include <SystemConfig.h>

// BlackLibs
#include "BlackADC.h"
#include "BlackDef.h"
#include "BlackGPIO.h"
#include "BlackI2C.h"
#include "BlackPWM.h"
#include "BlackSPI.h"

// BBB C++ API
#include <BeagleGPIO.h>
#include <BeaglePins.h>
#include <BeaglePWM.h>

// ROS
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "msl_actuator_msgs/BallHandleCmd.h"
#include "msl_actuator_msgs/HaveBallInfo.h"
#include "msl_actuator_msgs/IMUData.h"
#include "msl_actuator_msgs/MotionLight.h"
#include "msl_actuator_msgs/MotionBurst.h"
#include "msl_actuator_msgs/ShovelSelectCmd.h"
#include "msl_actuator_msgs/VisionRelocTrigger.h"
#include "process_manager/ProcessCommand.h"

#define TIMEDIFFUS(n,o) (((n).tv_usec-(o).tv_usec))
#define TIMEDIFFMS(n,o) (((n).tv_sec-(o).tv_sec)*1000+((n).tv_usec-(o).tv_usec)/1000)

#define	IMU_UPDATE_TIMEOUT			10		// ms
#define IMU_SEND_TIMEOUT			30		// ms

#define OpticalFlow_UPDATE_TIMEOUT	1		// ms
#define OpticalFlow_BURST_TIMEOUT	1		// ms


#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_CONFIG_H_ */
