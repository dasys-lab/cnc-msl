/*
 * imu.cpp
 *
 *  Created on: Mar 12, 2015
 *      Author: Lukas Will
 */


#include "imu.h"
#include "math.h"

using namespace BlackLib;

IMU::IMU(gpioName acc_P, gpioName gyro_P, gpioName mag_P, gpioName temp_P, BlackLib::BlackI2C *i2c_P) {
	i2c = i2c_P;

	i_acc = new BlackGPIO(acc_P, input, FastMode);
	i_gyro = new BlackGPIO(gyro_P, input, FastMode);
	i_mag = new BlackGPIO(mag_P, input, FastMode);
	i_temp = new BlackGPIO(temp_P, input, FastMode);

	temperature = 0;
}

IMU::~IMU() {
	delete i_acc;
	delete i_gyro;
	delete i_mag;
	delete i_temp;
}

void IMU::gReadBytes(uint8_t startAddress, uint8_t *dest, uint8_t count) {
	i2c->setDeviceAddress(ADR_G);
	for(int i=0; i<count; i++) {
		dest[i] = i2c->readByte(startAddress + i);
	}
}

void IMU::xmReadBytes(uint8_t startAddress, uint8_t *dest, uint8_t count) {
	i2c->setDeviceAddress(ADR_XM);
	for(int i=0; i<count; i++) {
		dest[i] = i2c->readByte(startAddress + i);
	}
}

void IMU::setHighPassFilterGyro(bool enable, uint8_t mode, uint8_t cutoff) {
	i2c->setDeviceAddress(ADR_G);

	uint8_t reg = i2c->readByte(CTRL_REG2_G);
	reg &= ~(0b00111111);

	i2c->writeByte(CTRL_REG1_G, reg | (mode << 4) | cutoff);

	if (enable) {
		reg = i2c->readByte(CTRL_REG5_G);
		reg &= ~(0b00010000);

		i2c->writeByte(CTRL_REG1_G, reg | 0x10);
	} else {
		reg = i2c->readByte(CTRL_REG5_G);
		reg &= ~(0b00010000);

		i2c->writeByte(CTRL_REG1_G, reg);
	}
}

bool IMU::init() {
	// Enable Accel
//	i2c->setDeviceAddress(ADR_XM);
//	i2c->writeByte(CTRL_REG1_XM, 0x67);			// Accel Frequecy: 100Hz

	initMagnet(true, MAG_RES_HIGH, MAG_RATE_25HZ, MAG_MFS_2GAUSS);

	// Enable Gyro
//	i2c->setDeviceAddress(ADR_G);
//	i2c->writeByte(CTRL_REG1_G, 0x0F);			// Gyro
//	i2c->writeByte(CTRL_REG2_G, 0x20);			// Test HPF-Mode
//	i2c->writeByte(CTRL_REG5_G, 0x10);			// High-Pass Filter enabled

	if(!this->whoami()) {
		return false;
	} else {
		return true;
	}
}

bool IMU::whoami() {
	uint8_t g, xm;

	i2c->setDeviceAddress(ADR_G);
	g = i2c->readByte(0x0f);

	i2c->setDeviceAddress(ADR_XM);
	xm = i2c->readByte(0x0f);

	if((g == WHO_AM_I_G) && (xm == WHO_AM_I_XM)) {
		return true;
	} else {
		return false;
	}
}

void IMU::initMagnet(bool temp, uint8_t res, uint8_t rate, uint8_t scale) {
	if (i2c->getDeviceAddress() != ADR_XM) {
		i2c->setDeviceAddress(ADR_XM);
	}

	uint8_t r;

	// Set CTRL-Reg 5 with magnetic resolution and rate, enable temp sensor
	r = i2c->readByte(CTRL_REG5_XM);
	r = (r & ~(0b10000000)) | (temp << 7);
	r = (r & ~(0b01100000)) | (res << 5);
	r = (r & ~(0b00011100)) | (rate << 2);
	i2c->writeByte(CTRL_REG5_XM, r);

	// Set CTRL-Reg 6 with magnetic scale
	r = i2c->readByte(CTRL_REG6_XM);
	r = (r & ~(0b01100000)) | (scale << 5);
	i2c->writeByte(CTRL_REG6_XM, r);

	// Set CTRL-Reg 7
	r = i2c->readByte(CTRL_REG7_XM);

	i2c->writeByte(CTRL_REG7_XM, r);

	switch (scale) {
		case MAG_MFS_2GAUSS:
			magnet.sense = MAG_SENSE_2GAUSS;
			break;

		case MAG_MFS_4GAUSS:
			magnet.sense = MAG_SENSE_4GAUSS;
			break;

		case MAG_MFS_8GAUSS:
			magnet.sense = MAG_SENSE_8GAUSS;
			break;

		case MAG_MFS_12GAUSS:
			magnet.sense = MAG_SENSE_12GAUSS;
			break;

		default:
			magnet.sense = MAG_SENSE_2GAUSS;
			break;
	}
}

void IMU::setupAccel(uint8_t range) {
	uint8_t reg;

	if (i2c->getDeviceAddress() != ADR_XM) {
		i2c->setDeviceAddress(ADR_XM);
	}

	reg = i2c->readByte(CTRL_REG2_XM);
	reg &= ~(0b00111000);

	i2c->writeByte(CTRL_REG2_XM, reg | range);
}

void IMU::setupGyro(uint8_t scale) {
	uint8_t reg;

	if (i2c->getDeviceAddress() != ADR_G) {
		i2c->setDeviceAddress(ADR_G);
	}

	reg = i2c->readByte(CTRL_REG4_G);
	reg &= ~(0b00110000);

	i2c->writeByte(CTRL_REG4_G, reg | scale);
}


void IMU::getAccel() {
	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	if (i2c->getDeviceAddress() != ADR_XM) {
		i2c->setDeviceAddress(ADR_XM);
	}
	for(int i=0; i<6; i++) {
		val[i] = i2c->readByte(ACCEL_OUT_X + i);
	}

	accel.x = (int16_t)(val[0] | ((int16_t)val[1] << 8)) * 9.81 * accel.sense;// - accel.xr;
	accel.y = (int16_t)(val[2] | ((int16_t)val[3] << 8)) * 9.81 * accel.sense;// - accel.yr;
	accel.z = (int16_t)(val[4] | ((int16_t)val[5] << 8)) * 9.81 * accel.sense;// - accel.zr;

//	accel.x = ((((int16_t) val[1] << 8) | val[0]) * 9.81 * accel.sense);// - accel.xr;
//	accel.y = ((((int16_t) val[3] << 8) | val[2]) * 9.81 * accel.sense);// - accel.yr;
//	accel.z = ((((int16_t) val[5] << 8) | val[4]) * 9.81 * accel.sense);// - accel.zr;
}


void IMU::getGyro() {
	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	if (i2c->getDeviceAddress() != ADR_G) {
		i2c->setDeviceAddress(ADR_G);
	}
	for(int i=0; i<6; i++) {
		val[i] = i2c->readByte(GYRO_OUT_X + i);
	}

	gyro.x = (int16_t)(val[0] | ((int16_t)val[1] << 8)) * 9.81 * gyro.sense;
	gyro.y = (int16_t)(val[2] | ((int16_t)val[3] << 8)) * 9.81 * gyro.sense;
	gyro.z = (int16_t)(val[4] | ((int16_t)val[5] << 8)) * 9.81 * gyro.sense;
}

void IMU::getMagnet() {
	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	if (i2c->getDeviceAddress() != ADR_XM) {
		i2c->setDeviceAddress(ADR_XM);
	}
	for(int i=0; i<6; i++) {
		val[i] = i2c->readByte(0x80 | (MAGNET_OUT_X + i));
	}
	magnet.x = (int16_t)(val[0] | ((int16_t)val[1] << 8));
	magnet.y = (int16_t)(val[2] | ((int16_t)val[3] << 8));
	magnet.z = (int16_t)(val[4] | ((int16_t)val[5] << 8));
}

void IMU::getTemp() {
	uint8_t val[2] = { 0x00, 0x00 };

	if (i2c->getDeviceAddress() != ADR_XM) {
		i2c->setDeviceAddress(ADR_XM);
	}
	for(int i=0; i<2; i++) {
		val[i] = i2c->readByte(TEMP_OUT + i);
	}

	// TODO Temperature Offset
	// temperature = 21.0 + (float) dof.temperature/8.;
	temperature = (((int16_t) val[1] << 12) | val[0] << 4 ) >> 4; // Temperature is a 12-bit signed integer
	temperature = (temperature-32)/1.8; // fareinheit -> celsiuis 
}

void IMU::updateData(timeval time_now) {
	//this->getAccel();
	//this->getGyro();
	this->getMagnet();
	//this->getTemp();

	std::cout << "X: " << magnet.x << std::endl;
	std::cout << "Y: " << magnet.y << std::endl;
	std::cout << "Z: " << magnet.z << std::endl;
	std::cout << "Sense: " << magnet.sense << std::endl;

	std::cout << "Winkel: " << atan2((double) magnet.y, (double) magnet.x) << std::endl;

	last_updated = time_now;
}

void IMU::sendData(timeval time_now, ros::Publisher *imuPub){
	msl_actuator_msgs::IMUData msg;

	msg.acceleration.x = accel.x;
	msg.acceleration.y = accel.y;
	msg.acceleration.z = accel.z;
	msg.accelSens = accel.sense;
	msg.gyro.x = gyro.x;
	msg.gyro.y = gyro.y;
	msg.gyro.z = gyro.z;
	msg.gyroSens = gyro.sense;
	msg.magnet.x = magnet.x;
	msg.magnet.y = magnet.y;
	msg.magnet.z = magnet.z;
	msg.magnetSens = magnet.sense;
	msg.temperature = temperature;
	msg.time = (unsigned long long)time_now.tv_sec*1000000 + time_now.tv_usec;

//	imuPub->publish(msg);
	last_sended = time_now;
}


