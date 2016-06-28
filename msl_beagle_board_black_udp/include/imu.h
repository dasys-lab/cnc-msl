/*
 * imu.h
 *
 *  Created on: Mar 12, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_

#include "includes.h"
#include "sensor.h"

const uint8_t ADR_G = 0x6B; // LSM9DS0
const uint8_t ADR_XM = 0x1D; // LSM9DS0

const uint8_t WHO_AM_I_G = 0xD4;
const uint8_t WHO_AM_I_XM = 0x49;

const uint8_t ACC_OUT_X = 0x28;
const uint8_t ACC_OUT_Y = 0x2A;
const uint8_t ACC_OUT_Z = 0x2C;
const uint8_t GYR_OUT_X = 0x28;
const uint8_t GYR_OUT_Y = 0x2A;
const uint8_t GYR_OUT_Z = 0x2C;
const uint8_t MAG_OUT_X = 0x08;
const uint8_t MAG_OUT_Y = 0x0A;
const uint8_t MAG_OUT_Z = 0x0C;
const uint8_t TEMP_OUT = 0x05;

const uint8_t CTRL_REG0_XM = 0x1F;
const uint8_t CTRL_REG1_XM = 0x20;
const uint8_t CTRL_REG2_XM = 0x21;
const uint8_t CTRL_REG3_XM = 0x22;
const uint8_t CTRL_REG4_XM = 0x23;
const uint8_t CTRL_REG5_XM = 0x24;
const uint8_t CTRL_REG6_XM = 0x25;
const uint8_t CTRL_REG7_XM = 0x26;

const uint8_t CTRL_REG1_G = 0x20;
const uint8_t CTRL_REG2_G = 0x21;
const uint8_t CTRL_REG3_G = 0x22;
const uint8_t CTRL_REG4_G = 0x23;
const uint8_t CTRL_REG5_G = 0x24;

const uint8_t MAG_RES_LOW = 0x00;
const uint8_t MAG_RES_HIGH = 0x03;

const uint8_t MAG_RATE_3HZ = 0x00;
const uint8_t MAG_RATE_6HZ = 0x01;
const uint8_t MAG_RATE_12HZ = 0x02;
const uint8_t MAG_RATE_25HZ = 0x03;
const uint8_t MAG_RATE_50HZ = 0x04;
const uint8_t MAG_RATE_100HZ = 0x05;

const uint8_t MAG_MFS_2GAUSS = 0x00;
const uint8_t MAG_MFS_4GAUSS = 0x01;
const uint8_t MAG_MFS_8GAUSS = 0x02;
const uint8_t MAG_MFS_12GAUSS = 0x03;

const uint8_t MAG_MODE_CONTINUOUS = 0x00;
const uint8_t MAG_MODE_SINGLE = 0x01;
const uint8_t MAG_MODE_POWERDOWN = 0x02;

const float MAG_SENSE_2GAUSS = 0.08;
const float MAG_SENSE_4GAUSS = 0.16;
const float MAG_SENSE_8GAUSS = 0.32;
const float MAG_SENSE_12GAUSS = 0.48;
const float TEMP_SENSE = 0.125;

const uint8_t ACC_RATE_POWERDOWN = 0x00;
const uint8_t ACC_RATE_3 = 0x01;
const uint8_t ACC_RATE_6 = 0x02;
const uint8_t ACC_RATE_12 = 0x03;
const uint8_t ACC_RATE_25 = 0x04;
const uint8_t ACC_RATE_50 = 0x05;
const uint8_t ACC_RATE_100 = 0x06;
const uint8_t ACC_RATE_200 = 0x07;
const uint8_t ACC_RATE_400 = 0x08;
const uint8_t ACC_RATE_800 = 0x09;
const uint8_t ACC_RATE_1600 = 0x0A;

const uint8_t ACC_ENABLE_X = 0x01;
const uint8_t ACC_ENABLE_Y = 0x02;
const uint8_t ACC_ENABLE_Z = 0x04;

const uint8_t ACC_AFS_2G = 0x00;
const uint8_t ACC_AFS_4G = 0x01;
const uint8_t ACC_AFS_6G = 0x02;
const uint8_t ACC_AFS_8G = 0x03;
const uint8_t ACC_AFS_16G = 0x04;

const float ACC_SENSE_2G = 0.061;
const float ACC_SENSE_4G = 0.122;
const float ACC_SENSE_6G = 0.183;
const float ACC_SENSE_8G = 0.244;
const float ACC_SENSE_16G = 0.732;

const uint8_t GYR_ENABLE_X = 0x01;
const uint8_t GYR_ENABLE_Y = 0x02;
const uint8_t GYR_ENABLE_Z = 0x04;

const uint8_t GYR_RATE_95 = 0x00;
const uint8_t GYR_RATE_190 = 0x01;
const uint8_t GYR_RATE_380 = 0x02;
const uint8_t GYR_RATE_760 = 0x03;

const uint8_t GYR_BW_00 = 0x00;
const uint8_t GYR_BW_01 = 0x01;
const uint8_t GYR_BW_10 = 0x02;
const uint8_t GYR_BW_11 = 0x03;

const uint8_t GYR_ENABLE_HPF = 0x10;
const uint8_t GYR_DISABLE_HPF = 0x10;

const uint8_t GYR_HPFMODE_NORMRST = 0x00;
const uint8_t GYR_HPFMODE_REF = 0x01;
const uint8_t GYR_HPFMODE_NORM = 0x02;
const uint8_t GYR_HPFMODE_AUTORST = 0x03;

//const uint8_t GYR_HPFCUT_		= 0x00;
//const uint8_t GYR_HPFCUT_		= 0x01;
//const uint8_t GYR_HPFCUT_		= 0x02;
//const uint8_t GYR_HPFCUT_		= 0x03;

const uint8_t GYR_FS_245DPS = 0x00;
const uint8_t GYR_FS_500DPS = 0x01;
const uint8_t GYR_FS_2000DPS = 0x10;
const uint8_t GYR_FS_2000DPS2 = 0x11;

const float GYR_SENSE_245DPS = 8.75;
const float GYR_SENSE_500DPS = 17.5;
const float GYR_SENSE_2000DPS = 70;

class IMU
{
public:
	IMU(const char *pin_names[], BlackLib::BlackI2C *i2c_P);
	~IMU();

	bool init();
	void getData(timeval time_now);
	msl_actuator_msgs::IMUData sendData(timeval time_now);

private:
	BlackLib::BlackI2C *i2c;
	BeagleGPIO *gpio;
	BeaglePins *pins;

	timeval last_updated;
	timeval last_sended;

	Sensor* gyr;
	Sensor* acc;
	Sensor* mag;
	int16_t temperature;

	bool whoAmI();
	void initAccel(uint8_t rate, uint8_t scale);
	void initGyro(uint8_t rate, uint8_t bandwidth, uint8_t scale);
	void initMagnet(uint8_t res, uint8_t rate, uint8_t scale);
	void initTemp(bool enable);
	void getOffsets();
	void getAccel();
	void getGyro();
	void getMagnet();
	void getTemp();
};

#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_IMU_H_ */
