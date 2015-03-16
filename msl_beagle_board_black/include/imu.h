/*
 * imu.h
 *
 *  Created on: Mar 12, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_

#include <stdint.h>

const uint8_t ADR_GYRO		= 0x69;
const uint8_t ADR_ACCEL		= 0x53;
const uint8_t ADR_MAGNET	= 0x1E;
const uint8_t ADR_THERMO	= 0x77;

const uint8_t GYRO_OUT_X			= 0x28;
const uint8_t GYRO_OUT_Y			= 0x2A;
const uint8_t GYRO_OUT_Z			= 0x2C;





#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_ */
