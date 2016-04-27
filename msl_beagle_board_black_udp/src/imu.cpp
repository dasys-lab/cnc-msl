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

bool IMU::init() {
	initAccel(ACC_RATE_25, ACC_AFS_2G);
	initGyro(GYR_RATE_95, GYR_BW_00, GYR_FS_500DPS);
	initMagnet(MAG_RES_LOW, MAG_RATE_12HZ, MAG_MFS_4GAUSS);
	initTemp(true);

	if(!this->whoami())
		return false;

	getOffsets();

	return true;
}

bool IMU::whoami() {
	uint8_t g, xm;

	i2c->setDeviceAddress(ADR_G);
	g = i2c->readByte(0x0f);

	i2c->setDeviceAddress(ADR_XM);
	xm = i2c->readByte(0x0f);

	if(!(g == WHO_AM_I_G) || !(xm == WHO_AM_I_XM))
		return false;

	return true;
}

void IMU::initAccel(uint8_t rate, uint8_t scale) {
	if (i2c->getDeviceAddress() != ADR_XM)
		i2c->setDeviceAddress(ADR_XM);

	uint8_t r;

	// Set CTRL-Reg 1 with acceleration rate and enable axis
	r = i2c->readByte(CTRL_REG1_XM);
	r = (r & ~(0b11110000)) | (rate << 4);
	r = (r & ~(0b00000111)) | ACC_ENABLE_X | ACC_ENABLE_Y | ACC_ENABLE_Z;
	i2c->writeByte(CTRL_REG1_XM, r);

	// Set CTRL-Reg 2 with acceleration scale
	r = i2c->readByte(CTRL_REG2_XM);
	r = (r & ~(0b00111000)) | (scale << 3);
	i2c->writeByte(CTRL_REG6_XM, r);

	switch (scale) {
		case ACC_AFS_2G:
			accel.sense = ACC_SENSE_2G;
			break;

		case ACC_AFS_4G:
			accel.sense = ACC_SENSE_4G;
			break;

		case ACC_AFS_6G:
			accel.sense = ACC_SENSE_6G;
			break;

		case ACC_AFS_8G:
			accel.sense = ACC_SENSE_8G;
			break;

		case ACC_AFS_16G:
			accel.sense = ACC_SENSE_16G;
			break;

		default:
			accel.sense = ACC_SENSE_2G;
			break;
	}
}

void IMU::initGyro(uint8_t rate, uint8_t bandwidth, uint8_t scale) {
	if (i2c->getDeviceAddress() != ADR_G)
		i2c->setDeviceAddress(ADR_G);

	uint8_t r;

	// Set CTRL-Reg 1 with rate, bandwidth and enable axis
	r = i2c->readByte(CTRL_REG1_G);
	r = (r & ~(0b11110000)) | (rate << 6) | (bandwidth << 4);
	r = (r & ~(0b00001000)) | (1 << 3);								// activate Gyro: Normal Mode
	r = (r & ~(0b00000111)) | GYR_ENABLE_X | GYR_ENABLE_Y | GYR_ENABLE_Z;
	i2c->writeByte(CTRL_REG1_G, r);

	// Set CTRL-Reg 2 for high-pass filter (actually disabled)
	r = i2c->readByte(CTRL_REG2_G);
	r = (r & ~(0b00110000)) | (GYR_HPFMODE_NORMRST << 4);
	r = (r & ~(0b00001111)) | 0x00;
	i2c->writeByte(CTRL_REG2_G, r);

	// Set CTRL-Reg 4 with scale
	r = i2c->readByte(CTRL_REG4_G);
	r = (r & ~(0b00110000)) | (scale << 4);
	i2c->writeByte(CTRL_REG4_G, r);

	// Set CTRL-Reg 5 with scale
	r = i2c->readByte(CTRL_REG5_G);
	r = (r & ~(0b00010000)) | GYR_DISABLE_HPF;
	i2c->writeByte(CTRL_REG5_G, r);

	switch (scale) {
		case GYR_FS_245DPS:
			gyro.sense = GYR_SENSE_245DPS;
			break;

		case GYR_FS_500DPS:
			gyro.sense = GYR_SENSE_500DPS;
			break;

		case GYR_FS_2000DPS:
			gyro.sense = GYR_SENSE_2000DPS;
			break;

		case GYR_FS_2000DPS2:
			gyro.sense = GYR_SENSE_2000DPS;
			break;

		default:
			gyro.sense = GYR_SENSE_245DPS;
			break;
	}
}

void IMU::initMagnet(uint8_t res, uint8_t rate, uint8_t scale) {
	if (i2c->getDeviceAddress() != ADR_XM)
		i2c->setDeviceAddress(ADR_XM);

	uint8_t r;

	// Set CTRL-Reg 5 with magnetic resolution and rate
	r = i2c->readByte(CTRL_REG5_XM);
	r = (r & ~(0b01100000)) | (res << 5);
	r = (r & ~(0b00011100)) | (rate << 2);
	i2c->writeByte(CTRL_REG5_XM, r);

	// Set CTRL-Reg 6 with magnetic scale
	r = i2c->readByte(CTRL_REG6_XM);
	r = (r & ~(0b01100000)) | (scale << 5);
	i2c->writeByte(CTRL_REG6_XM, r);

	// Set CTRL-Reg 7 with continuous conversion mode
	r = i2c->readByte(CTRL_REG7_XM);
	r = (r & ~(0b00000011)) | MAG_MODE_CONTINUOUS;
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

void IMU::initTemp(bool enable) {
	uint8_t r;

	// enable/disable temperature sensor
	r = i2c->readByte(CTRL_REG5_XM);
	r = (r & ~(0b10000000)) | (enable << 7);
	i2c->writeByte(CTRL_REG5_XM, r);
}


void IMU::getOffsets() {
	this->getAccel();
	this->getGyro();

	accel.xoff = accel.xraw;
	accel.yoff = accel.yraw;
	accel.zoff = (1000 / accel.sense) - accel.zraw;
}

void IMU::getAccel() {
	if (i2c->getDeviceAddress() != ADR_XM)
		i2c->setDeviceAddress(ADR_XM);

	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	i2c->readBlock(0x80 | ACC_OUT_X, val, 6);		// 0x80 needed for readBlock() function

	accel.xraw = (int16_t)(val[0] | ((int16_t)val[1] << 8));
	accel.yraw = (int16_t)(val[2] | ((int16_t)val[3] << 8));
	accel.zraw = (int16_t)(val[4] | ((int16_t)val[5] << 8));

	// conversion from acceleration of gravity in [mm/s^2] to acceleration in [m/s^2]
	accel.x = (accel.xraw - accel.xoff) * 9.81 * accel.sense / 1000;
	accel.y = (accel.yraw - accel.yoff) * 9.81 * accel.sense / 1000;
	accel.z = (accel.zraw - accel.zoff) * 9.81 * accel.sense / 1000;
}


void IMU::getGyro() {
	if (i2c->getDeviceAddress() != ADR_G)
		i2c->setDeviceAddress(ADR_G);

	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	i2c->readBlock(0x80 | GYR_OUT_X, val, 6);		// 0x80 needed for readBlock() function

	gyro.xraw = (int16_t)(val[0] | ((int16_t)val[1] << 8));
	gyro.yraw = (int16_t)(val[2] | ((int16_t)val[3] << 8));
	gyro.zraw = (int16_t)(val[4] | ((int16_t)val[5] << 8));

	// conversion from milli degree per second in [mdps] to degree per second [dps]
	gyro.x = gyro.xraw * gyro.sense / 1000;
	gyro.y = gyro.yraw * gyro.sense / 1000;
	gyro.z = gyro.zraw * gyro.sense / 1000;
}

void IMU::getMagnet() {
	if (i2c->getDeviceAddress() != ADR_XM)
		i2c->setDeviceAddress(ADR_XM);

	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	i2c->readBlock(0x80 | MAG_OUT_X, val, 6);		// 0x80 needed for readBlock() function

	magnet.xraw = (int16_t)(val[0] | ((int16_t)val[1] << 8));
	magnet.yraw = (int16_t)(val[2] | ((int16_t)val[3] << 8));
	magnet.zraw = (int16_t)(val[4] | ((int16_t)val[5] << 8));
	
	// conversion from milli gauss [mgauss] to radiant and degree
	compass.angle_rad = atan2((double) magnet.yraw, (double) magnet.xraw);
	compass.angle_deg = compass.angle_rad * 180 / M_PI;
}

void IMU::getTemp() {
	if (i2c->getDeviceAddress() != ADR_XM)
		i2c->setDeviceAddress(ADR_XM);

	uint8_t val[2] = { 0x00, 0x00 };
	for(int i=0; i<2; i++)
		val[i] = i2c->readByte(TEMP_OUT + i);

	// TODO Temperature Offset
	int16_t temp = (int16_t)(val[0] | ((int16_t)val[1] << 8));
	temperature = (temp - 32) / 1.8; // Fahreinheit -> Celsiuis
}

void IMU::getData(timeval time_now) {
	this->getAccel();
	this->getGyro();
	this->getMagnet();
	this->getTemp();

	std::cout << "ACC X: " << accel.x << std::endl;
	std::cout << "ACC Y: " << accel.y << std::endl;
	std::cout << "ACC Z: " << accel.z << std::endl;

	std::cout << "GYR X: " << gyro.xraw << std::endl;
	std::cout << "GYR Y: " << gyro.yraw << std::endl;
	std::cout << "GYR Z: " << gyro.zraw << std::endl;

	std::cout << "Angle rad: " << compass.angle_rad << std::endl;
	std::cout << "Angle deg: " << compass.angle_deg << std::endl;

	std::cout << "TEMP: " << temperature << std::endl;

	last_updated = time_now;
}

void IMU::sendData(timeval time_now){
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

	last_sended = time_now;
}


