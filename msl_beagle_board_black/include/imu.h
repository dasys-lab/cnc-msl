/*
 * imu.h
 *
 *  Created on: Mar 12, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_

#include "config.h"


const uint8_t ADR_G				= 0x6B;		// LSM9DS0
const uint8_t ADR_XM			= 0x1D;		// LSM9DS0

const uint8_t WHO_AM_I_G		= 0xD4;
const uint8_t WHO_AM_I_XM		= 0x49;

const uint8_t ACCEL_OUT_X			= 0x28;
const uint8_t ACCEL_OUT_Y			= 0x2A;
const uint8_t ACCEL_OUT_Z			= 0x2C;
const uint8_t GYRO_OUT_X			= 0x28;
const uint8_t GYRO_OUT_Y			= 0x2A;
const uint8_t GYRO_OUT_Z			= 0x2C;
const uint8_t MAGNET_OUT_X			= 0x08;
const uint8_t MAGNET_OUT_Y			= 0x0A;
const uint8_t MAGNET_OUT_Z			= 0x0C;
const uint8_t TEMP_OUT				= 0x05;

const uint8_t CTRL_REG0_XM			= 0x1F;
const uint8_t CTRL_REG1_XM			= 0x20;
const uint8_t CTRL_REG2_XM			= 0x21;
const uint8_t CTRL_REG3_XM			= 0x22;
const uint8_t CTRL_REG4_XM			= 0x23;
const uint8_t CTRL_REG5_XM			= 0x24;
const uint8_t CTRL_REG6_XM			= 0x25;
const uint8_t CTRL_REG7_XM			= 0x26;

const uint8_t CTRL_REG1_G			= 0x20;
const uint8_t CTRL_REG2_G			= 0x21;
const uint8_t CTRL_REG3_G			= 0x22;
const uint8_t CTRL_REG4_G			= 0x23;
const uint8_t CTRL_REG5_G			= 0x24;

const uint8_t ACC_AFS_2G			= 0x00;
const uint8_t ACC_AFS_4G			= 0x08;
const uint8_t ACC_AFS_6G			= 0x10;
const uint8_t ACC_AFS_8G			= 0x18;
const uint8_t ACC_AFS_16G			= 0x20;
const uint8_t GYR_FS_245DPS			= 0x00;
const uint8_t GYR_FS_500DPS			= 0x01;
const uint8_t GYR_FS_2000DPS		= 0x10;
const uint8_t GYR_FS_2000DPS2		= 0x11;
const uint8_t MAG_MDR_2GAUSS		= 0x00;
const uint8_t MAG_MDR_4GAUSS		= 0x20;
const uint8_t MAG_MDR_6GAUSS		= 0x40;
const uint8_t MAG_MDR_8GAUSS		= 0x60;




class IMU {
	private:
		BlackLib::BlackI2C *i2c;
		timeval		last_sended, last_updated;
		int16_t		temperature;

		struct Koordinaten {
			int16_t	x, y, z;
		} gyro, accel, magnet;

		bool		whoami();
		void		setupAccel(uint8_t range);
		void		setupGyro(uint8_t scale);
		void		setupMagnet(uint8_t scale);
		void		getAccel();
		void		getGyro();
		void		getMagnet();
		void		getTemp();

	public:
					IMU(BlackLib::BlackI2C *i2c_P);
					~IMU();
		void		init();
		void		updateData(timeval time_now);
		void		sendData(timeval time_now, ros::Publisher *imuPub);

};



#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_ */
