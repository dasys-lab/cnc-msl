/*
 * imu.cpp
 *
 *  Created on: Mar 12, 2015
 *      Author: Lukas Will
 */


#include "imu.h"


IMU::IMU(BlackLib::BlackI2C *i2c_P) {
	i2c = i2c_P;
	temperature = 0;

	this->init();
}

IMU::~IMU() {

}

void IMU::getAccel() {
	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	i2c->writeByte(ADR_ACCEL, ACCEL_OUT_X);
	uint8_t check = i2c->readBlock(ADR_ACCEL, val, sizeof(val));

	std::cout << "Check: " << check << std::endl;

	accel.x = ((int16_t) val[1] << 8) | val[0];
	accel.y = ((int16_t) val[3] << 8) | val[2];
	accel.z = ((int16_t) val[5] << 8) | val[4];
}

int16_t IMU::getAccelX() {
	uint8_t val[2] = { 0x00, 0x00 };

	i2c->writeByte(ADR_ACCEL, ACCEL_OUT_X);
	uint8_t check = i2c->readBlock(ADR_ACCEL, val, sizeof(val));

	return ((int16_t) val[1] << 8) | val[0];
}

int16_t IMU::getAccelY() {
	uint8_t val[2] = { 0x00, 0x00 };

	i2c->writeByte(ADR_ACCEL, ACCEL_OUT_Y);
	uint8_t check = i2c->readBlock(ADR_ACCEL, val, sizeof(val));

	return ((int16_t) val[1] << 8) | val[0];
}

int16_t IMU::getAccelZ() {
	uint8_t val[2] = { 0x00, 0x00 };

	i2c->writeByte(ADR_ACCEL, ACCEL_OUT_Z);
	uint8_t check = i2c->readBlock(ADR_ACCEL, val, sizeof(val));

	return ((int16_t) val[1] << 8) | val[0];
}

void IMU::getGyro() {
	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	i2c->writeByte(ADR_GYRO, GYRO_OUT_X);
	uint8_t check = i2c->readBlock(ADR_GYRO, val, sizeof(val));

	gyro.x = ((int16_t) val[1] << 8) | val[0];
	gyro.y = ((int16_t) val[3] << 8) | val[2];
	gyro.z = ((int16_t) val[5] << 8) | val[4];
}

int16_t IMU::getGyroX() {
	uint8_t val[2] = { 0x00, 0x00 };

	i2c->writeByte(ADR_GYRO, GYRO_OUT_X);
	uint8_t check = i2c->readBlock(ADR_GYRO, val, sizeof(val));

	return ((int16_t) val[1] << 8) | val[0];
}

int16_t IMU::getGyroY() {
	uint8_t val[2] = { 0x00, 0x00 };

	i2c->writeByte(ADR_GYRO, GYRO_OUT_Y);
	uint8_t check = i2c->readBlock(ADR_GYRO, val, sizeof(val));

	return ((int16_t) val[1] << 8) | val[0];
}

int16_t IMU::getGyroZ() {
	uint8_t val[2] = { 0x00, 0x00 };

	i2c->writeByte(ADR_GYRO, GYRO_OUT_Z);
	uint8_t check = i2c->readBlock(ADR_GYRO, val, sizeof(val));

	return ((int16_t) val[1] << 8) | val[0];
}

void IMU::getMagnet() {
	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	i2c->writeByte(ADR_MAGNET, MAGNET_OUT_X);
	uint8_t check = i2c->readBlock(ADR_MAGNET, val, sizeof(val));

	magnet.x = ((int16_t) val[1] << 8) | val[0];
	magnet.y = ((int16_t) val[3] << 8) | val[2];
	magnet.z = ((int16_t) val[5] << 8) | val[4];
}

int16_t IMU::getMagnetX() {
	uint8_t val[2] = { 0x00, 0x00 };

	i2c->writeByte(ADR_MAGNET, MAGNET_OUT_X);
	uint8_t check = i2c->readBlock(ADR_MAGNET, val, sizeof(val));

	return ((int16_t) val[1] << 8) | val[0];
}

int16_t IMU::getMagnetY() {
	uint8_t val[2] = { 0x00, 0x00 };

	i2c->writeByte(ADR_MAGNET, MAGNET_OUT_Y);
	uint8_t check = i2c->readBlock(ADR_MAGNET, val, sizeof(val));

	return ((int16_t) val[1] << 8) | val[0];
}

int16_t IMU::getMagnetZ() {
	uint8_t val[2] = { 0x00, 0x00 };

	i2c->writeByte(ADR_MAGNET, MAGNET_OUT_Z);
	uint8_t check = i2c->readBlock(ADR_MAGNET, val, sizeof(val));

	return ((int16_t) val[1] << 8) | val[0];
}

void IMU::getTemp() {
	uint8_t val[2] = { 0x00, 0x00 };

	i2c->writeByte(ADR_TEMP, TEMP_OUT);
	uint8_t check = i2c->readBlock(ADR_TEMP, val, sizeof(val));

	temperature = ((int16_t) val[1] << 8) | val[0];
}

void IMU::setupAccel(uint8_t range) {
	uint8_t reg, val[2];

	i2c->writeByte(ADR_ACCEL, CTRL_REG2_XM);
	reg = i2c->readByte(ADR_ACCEL);
	reg &= ~(0b00111000);

	val[0] = CTRL_REG2_XM;
	val[1] = reg | range;
	i2c->writeBlock(ADR_ACCEL, val, sizeof(val));
}

void IMU::setupGyro(uint8_t scale) {
	uint8_t reg, val[2];

	i2c->writeByte(ADR_GYRO, CTRL_REG4_G);
	reg = i2c->readByte(ADR_GYRO);
	reg &= ~(0b00110000);

	val[0] = CTRL_REG4_G;
	val[1] = reg | scale;
	i2c->writeBlock(ADR_GYRO, val, sizeof(val));
}

void IMU::setupMagnet(uint8_t scale) {
	uint8_t reg, val[2];

	i2c->writeByte(ADR_MAGNET, CTRL_REG6_XM);
	reg = i2c->readByte(ADR_MAGNET);
	reg &= ~(0b01100000);

	val[0] = CTRL_REG6_XM;
	val[1] = reg | scale;
	i2c->writeBlock(ADR_MAGNET, val, sizeof(val));
}

void IMU::init() {
	uint8_t val[2];

	// Enable Accel
	val[0] = CTRL_REG1_XM;
	val[1] = 0x67;				// Accel Frequecy: 100Hz
	i2c->writeBlock(ADR_ACCEL, val, sizeof(val));

	// Enable Magnet & Temp
	val[0] = CTRL_REG5_XM;
	val[1] = 0xF4;				// Magnet Frequecy: 100Hz & high resolution
	i2c->writeBlock(ADR_MAGNET, val, sizeof(val));

	// Enable Gyro
	val[0] = CTRL_REG1_G;
	val[1] = 0x0F;
	i2c->writeBlock(ADR_GYRO, val, sizeof(val));

	this->setupAccel(ACC_AFS_4G);
	this->setupGyro(GYR_FS_2000DPS);
	this->setupMagnet(MAG_MDR_4GAUSS);
}

void IMU::updateData(timeval time_now) {
	if(TIMEDIFFMS(time_now, last_updated) > IMU_UPDATE_TIMEOUT) {
		this->getAccel();
		this->getGyro();
		this->getMagnet();
		this->getTemp();

		last_updated = time_now;

		std::cout << "Accel: " << accel.x << " - " << accel.y << " - " << accel.z << std::endl;
		std::cout << "Gyro: " << gyro.x << " - " << gyro.y << " - " << gyro.z << std::endl;
		std::cout << "Magnet: " << magnet.x << " - " << magnet.y << " - " << magnet.z << std::endl;
		std::cout << "Temp: " << temperature << std::endl;
	}
}

void IMU::sendData(timeval time_now, ros::Publisher *imuPub){
	if(TIMEDIFFMS(time_now, last_sended) > IMU_SEND_TIMEOUT) {
		// x-, y- und z-Werte von ACCEL, GYRO und MAGNET publishen
		// Temperatur publishen

		/*msl_actuator_msgs::IMUInfo msg;

		msg.accel = accel;
		msg.gyro = gyro;
		msg.magnet = magnet;
		msg.temperature = temperature;

		imuPub->publish(msg);*/
		last_sended = time_now;
	}
}
