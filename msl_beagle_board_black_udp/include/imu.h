/*
 * imu.h
 *
 *  Created on: Mar 12, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_

#include "includes.h"


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
const uint8_t MAG_MDR_2GAUSS		= 0x00;
const uint8_t MAG_MDR_4GAUSS		= 0x20;
const uint8_t MAG_MDR_8GAUSS		= 0x40;
const uint8_t MAG_MDR_12GAUSS		= 0x60;

const float ACC_2G_SENSE			= 0.061;
const float ACC_4G_SENSE			= 0.122;
const float ACC_6G_SENSE			= 0.183;
const float ACC_8G_SENSE			= 0.244;
const float ACC_16G_SENSE			= 0.732;
const float GYR_245DPS_SENSE		= 8.75;
const float GYR_500DPS_SENSE		= 17.5;
const float GYR_2000DPS_SENSE		= 70;
const float MAG_2GAUSS_SENSE		= 0.08;
const float MAG_4GAUSS_SENSE		= 0.16;
const float MAG_8GAUSS_SENSE		= 0.32;
const float MAG_12GAUSS_SENSE		= 0.48;
const float TEMP_SENSE				= 0.125;




class IMU {
	private:
		BlackLib::BlackGPIO *i_acc, *i_gyro, *i_mag, *i_temp;
		BlackLib::BlackI2C *i2c;
		timeval		last_sended, last_updated;
		int16_t		temperature;

		struct Koordinaten {
			float	x, y, z, sense;
			float	xr, yr, zr;
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
					IMU(BlackLib::gpioName acc_P, BlackLib::gpioName gyro_P, BlackLib::gpioName mag_P, BlackLib::gpioName temp_P, BlackLib::BlackI2C *i2c_P);
					~IMU();


		void		gReadBytes(uint8_t startAddress, uint8_t *dest, uint8_t count);
		void		xmReadBytes(uint8_t startAddress, uint8_t *dest, uint8_t count);

		void		setDataRateGyro(uint8_t datarate, uint8_t bandwidth);
		void		setHighPassFilterGyro(bool enable, uint8_t mode, uint8_t cutoff);
		void		setScaleGyro(uint8_t scale);
		void		enableGyro(bool state);


		bool		init();
		void		setRefAccel();
		void		setRefGyro();
		void		updateData(timeval time_now);
		void		sendData(timeval time_now, ros::Publisher *imuPub);
};



#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_ */
