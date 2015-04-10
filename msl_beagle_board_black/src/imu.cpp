/*
 * imu.cpp
 *
 *  Created on: Mar 12, 2015
 *      Author: Lukas Will
 */


#include "imu.h"



void Imu::getGyro() {
	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	int16_t value = 0;

	i2c->writeByte(ADR_GYRO, GYRO_OUT_X);
	val = i2c->readBlock(ADR_GYRO, val, sizeof(val));

	gyro.x = ((int16_t) val[2] << 8) | val[1];
	gyro.y = ((int16_t) val[4] << 8) | val[3];
	gyro.z = ((int16_t) val[6] << 8) | val[5];
}

int16_t Imu::getGyroX() {
	uint8_t val[2] = { 0x00, 0x00 };
	int16_t value = 0;

	i2c->writeByte(ADR_GYRO, GYRO_OUT_X);
	val = i2c->readBlock(ADR_GYRO, val, sizeof(val));

	return ((int16_t) val[2] << 8) | val[1];
}

int16_t Imu::getGyroY() {
	uint8_t val[2] = { 0x00, 0x00 };
	int16_t value = 0;

	i2c->writeByte(ADR_GYRO, GYRO_OUT_Y);
	val = i2c->readBlock(ADR_GYRO, val, sizeof(val));

	return ((int16_t) val[2] << 8) | val[1];
}

int16_t Imu::getGyroZ() {
	uint8_t val[2] = { 0x00, 0x00 };
	int16_t value = 0;

	i2c->writeByte(ADR_GYRO, GYRO_OUT_Z);
	val = i2c->readBlock(ADR_GYRO, val, sizeof(val));

	return ((int16_t) val[2] << 8) | val[1];
}

void Imu::getMagnet() {
	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	int16_t value = 0;

	i2c->writeByte(ADR_MAGNET, MAGNET_OUT_X);
	val = i2c->readBlock(ADR_MAGNET, val, sizeof(val));

	magnet.x = ((int16_t) val[2] << 8) | val[1];
	magnet.y = ((int16_t) val[4] << 8) | val[3];
	magnet.z = ((int16_t) val[6] << 8) | val[5];
}

int16_t Imu::getMagnetX() {
	uint8_t val[2] = { 0x00, 0x00 };
	int16_t value = 0;

	i2c->writeByte(ADR_MAGNET, MAGNET_OUT_X);
	val = i2c->readBlock(ADR_MAGNET, val, sizeof(val));

	return ((int16_t) val[2] << 8) | val[1];
}

int16_t Imu::getMagnetY() {
	uint8_t val[2] = { 0x00, 0x00 };
	int16_t value = 0;

	i2c->writeByte(ADR_MAGNET, MAGNET_OUT_Y);
	val = i2c->readBlock(ADR_MAGNET, val, sizeof(val));

	return ((int16_t) val[2] << 8) | val[1];
}

int16_t Imu::getMagnetZ() {
	uint8_t val[2] = { 0x00, 0x00 };
	int16_t value = 0;

	i2c->writeByte(ADR_MAGNET, MAGNET_OUT_Z);
	val = i2c->readBlock(ADR_MAGNET, val, sizeof(val));

	return ((int16_t) val[2] << 8) | val[1];
}

void Imu::getAccel() {
	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	int16_t value = 0;

	i2c->writeByte(ADR_ACCEL, ACCEL_OUT_X);
	val = i2c->readBlock(ADR_ACCEL, val, sizeof(val));

	accel.x = ((int16_t) val[2] << 8) | val[1];
	accel.y = ((int16_t) val[4] << 8) | val[3];
	accel.z = ((int16_t) val[6] << 8) | val[5];
}

int16_t Imu::getAccelX() {
	uint8_t val[2] = { 0x00, 0x00 };
	int16_t value = 0;

	i2c->writeByte(ADR_ACCEL, ACCEL_OUT_X);
	val = i2c->readBlock(ADR_ACCEL, val, sizeof(val));

	return ((int16_t) val[2] << 8) | val[1];
}

int16_t Imu::getAccelY() {
	uint8_t val[2] = { 0x00, 0x00 };
	int16_t value = 0;

	i2c->writeByte(ADR_ACCEL, ACCEL_OUT_Y);
	val = i2c->readBlock(ADR_ACCEL, val, sizeof(val));

	return ((int16_t) val[2] << 8) | val[1];
}

int16_t Imu::getAccelZ() {
	uint8_t val[2] = { 0x00, 0x00 };
	int16_t value = 0;

	i2c->writeByte(ADR_ACCEL, ACCEL_OUT_Z);
	val = i2c->readBlock(ADR_ACCEL, val, sizeof(val));

	return ((int16_t) val[2] << 8) | val[1];
}

void Imu::updateData(timeval time_now) {
	if(TIMEDIFFMS(time_now, ) > TIMEOUT) {
		this->getAccel();
		this->getGyro();
		this->getMagnet();
	}
}

void Imu::sendData(timeval time_now, ros::Publisher *imuPub){
	if(TIMEDIFFMS(time_now, ) > TIMEOUT) {
		// x-, y- und z-Werte von ACCEL, GYRO und MAGNET publishen
	}
}
