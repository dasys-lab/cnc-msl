/*
 * imu.cpp
 *
 *  Created on: Mar 12, 2015
 *      Author: Lukas Will
 */


#include "imu.h"

#include "BlackI2C"



void getGyro (BlackLib::BlackI2C &i2c, int16_t *read) {
	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	int16_t value = 0;

	i2c.writeByte(ADR_GYRO, GYRO_OUT_X);
	val = i2c.readBlock(ADR_GYRO, val, sizeof(val));

	read[0] = ((int16_t) val[2] << 8) | val[1];
	read[1] = ((int16_t) val[4] << 8) | val[3];
	read[2] = ((int16_t) val[6] << 8) | val[5];
}

int16_t getGyroX (BlackLib::BlackI2C &i2c) {
	uint8_t val[2] = { 0x00, 0x00 };
	int16_t value = 0;

	i2c.writeByte(ADR_GYRO, GYRO_OUT_X);
	val = i2c.readBlock(ADR_GYRO, val, sizeof(val));

	return ((int16_t) val[2] << 8) | val[1];
}

int16_t getGyroY (BlackLib::BlackI2C &i2c) {
	uint8_t val[2] = { 0x00, 0x00 };
	int16_t value = 0;

	i2c.writeByte(ADR_GYRO, GYRO_OUT_Y);
	val = i2c.readBlock(ADR_GYRO, val, sizeof(val));

	return ((int16_t) val[2] << 8) | val[1];
}

int16_t getGyroZ (BlackLib::BlackI2C &i2c) {
	uint8_t val[2] = { 0x00, 0x00 };
	int16_t value = 0;

	i2c.writeByte(ADR_GYRO, GYRO_OUT_Z);
	val = i2c.readBlock(ADR_GYRO, val, sizeof(val));

	return ((int16_t) val[2] << 8) | val[1];
}
