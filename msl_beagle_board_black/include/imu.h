/*
 * imu.h
 *
 *  Created on: Mar 12, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_

#include "config.h"

//const uint8_t ADR_GYRO		= 0x69;			// GY-80
//const uint8_t ADR_ACCEL		= 0x53;			// GY-80
//const uint8_t ADR_MAGNET	= 0x1E;			// GY-80
//const uint8_t ADR_THERMO	= 0x77;			// GY-80


const uint8_t ADR_GYRO			= 0xD4;		// LSM9DS0
const uint8_t ADR_MAGNET		= 0x49;		// LSM9DS0
const uint8_t ADR_ACCEL			= 0x49;		// LSM9DS0

const uint8_t GYRO_OUT_X			= 0x28;
const uint8_t GYRO_OUT_Y			= 0x2A;
const uint8_t GYRO_OUT_Z			= 0x2C;

const uint8_t MAGNET_OUT_X			= 0x08;
const uint8_t MAGNET_OUT_Y			= 0x0A;
const uint8_t MAGNET_OUT_Z			= 0x0C;

const uint8_t ACCEL_OUT_X			= 0x28;
const uint8_t ACCEL_OUT_Y			= 0x2A;
const uint8_t ACCEL_OUT_Z			= 0x2C;


class Imu {
	private:
		BlackLib::BlackI2C *i2c;

		struct Koordinaten {
			uint16_t	x, y, z;
		} gyro, accel, magnet;



		void		getGyro();
		int16_t		getGyroX();
		int16_t		getGyroY();
		int16_t		getGyroZ();

		void		getMagnet();
		int16_t		getMagnetX();
		int16_t		getMagnetY();
		int16_t		getMagnetZ();

		void		getAccel();
		int16_t		getAccelX();
		int16_t		getAccelY();
		int16_t		getAccelZ();

	public:
					Imu(BlackLib::BlackI2C *i2c);
					~Imu();
		void		updateData();
		void		sendData();

};



#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_ */
