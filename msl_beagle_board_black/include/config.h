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

// ROS
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "msl_actuator_msgs/BallCatchCmd.h"
#include "msl_actuator_msgs/BallHandleCmd.h"
#include "msl_actuator_msgs/HaveBallInfo.h"
#include "msl_actuator_msgs/MotionLight.h"
#include "msl_actuator_msgs/MotionBurst.h"
#include "msl_actuator_msgs/ShovelSelectCmd.h"
#include "msl_actuator_msgs/VisionRelocTrigger.h"

// BlackLibs
#include "BlackADC.h"
#include "BlackDef.h"
#include "BlackGPIO.h"
#include "BlackI2C.h"
#include "BlackPWM.h"
#include "BlackSPI.h"

// Threads
#include <thread>
#include <mutex>
#include <condition_variable>



#define TIMEDIFFUS(n,o) (((n).tv_usec-(o).tv_usec))
#define TIMEDIFFMS(n,o) (((n).tv_sec-(o).tv_sec)*1000+((n).tv_usec-(o).tv_usec)/1000)

#define PING_TIMEOUT				1000	// ms

#define BallHandle_TIMEOUT			1000	// ms
#define BallHandle_PWM_STEP_SIZE	50

#define	IMU_UPDATE_TIMEOUT			10		// ms
#define IMU_SEND_TIMEOUT			30		// ms

#define OpticalFlow_UPDATE_TIMEOUT	1		// ms
#define OpticalFlow_BURST_TIMEOUT	1		// ms

#define ShovelSelect_TIMEOUT		250			// ms
#define ShovelSelect_PASSING		1300000		// 1000 - PWM ( 1,30ms / 20ms )
#define ShovelSelect_NORMAL			1950000		// 2000 - PWM ( 1,95ms / 20ms )
#define ShovelSelect_PERIOD			20000000	// Servo Period Time 20ms


// Entscheidungsschwelle fuer Ball (0 bis 65000)
const uint16_t LIGHTBARRIER_THRESHOLD = 500;


#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_CONFIG_H_ */
