/*
 * imu.cpp
 *
 *  Created on: Mar 12, 2015
 *      Author: Lukas Will
 */


#include "imu.h"

using namespace BlackLib;

IMU::IMU(const char *pin_names[], BlackLib::BlackI2C *i2c_P) {
	i2c = i2c_P;

	gpio = BeagleGPIO::getInstance();
	pins = gpio->claim((char**) pin_names, 4);

	acc = new Sensor();
	gyr = new Sensor();
	mag = new Sensor();

	temperature = 0;
}

IMU::~IMU() {
	delete gpio;
	delete acc;
	delete gyr;
	delete mag;
}



bool IMU::init() {
	initAccel(ACC_RATE_25, ACC_AFS_2G);
	initGyro(GYR_RATE_95, GYR_BW_00, GYR_FS_500DPS);
	initMagnet(MAG_RES_LOW, MAG_RATE_12HZ, MAG_MFS_4GAUSS);
	initTemp(true);

	if(!this->whoAmI())
		return false;

	return true;
}

bool IMU::whoAmI() {
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
			acc->sense = ACC_SENSE_2G;
			break;

		case ACC_AFS_4G:
			acc->sense = ACC_SENSE_4G;
			break;

		case ACC_AFS_6G:
			acc->sense = ACC_SENSE_6G;
			break;

		case ACC_AFS_8G:
			acc->sense = ACC_SENSE_8G;
			break;

		case ACC_AFS_16G:
			acc->sense = ACC_SENSE_16G;
			break;

		default:
			acc->sense = ACC_SENSE_2G;
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
			gyr->sense = GYR_SENSE_245DPS;
			break;

		case GYR_FS_500DPS:
			gyr->sense = GYR_SENSE_500DPS;
			break;

		case GYR_FS_2000DPS:
			gyr->sense = GYR_SENSE_2000DPS;
			break;

		case GYR_FS_2000DPS2:
			gyr->sense = GYR_SENSE_2000DPS;
			break;

		default:
			gyr->sense = GYR_SENSE_245DPS;
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
			mag->sense = MAG_SENSE_2GAUSS;
			break;

		case MAG_MFS_4GAUSS:
			mag->sense = MAG_SENSE_4GAUSS;
			break;

		case MAG_MFS_8GAUSS:
			mag->sense = MAG_SENSE_8GAUSS;
			break;

		case MAG_MFS_12GAUSS:
			mag->sense = MAG_SENSE_12GAUSS;
			break;

		default:
			mag->sense = MAG_SENSE_2GAUSS;
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

void IMU::getAccel() {
	if (i2c->getDeviceAddress() != ADR_XM)
		i2c->setDeviceAddress(ADR_XM);

	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	i2c->readBlock(0x80 | ACC_OUT_X, val, 6);		// 0x80 needed for readBlock() function

	shared_ptr<geometry::CNPoint3D> point = make_shared<geometry::CNPoint3D>();

	// get data and conversion from acceleration of gravity [mm/s^2] to acceleration [m/s^2]
	point->x = (int16_t)(val[0] | ((int16_t)val[1] << 8));
	point->y = (int16_t)(val[2] | ((int16_t)val[3] << 8));
	point->z = (int16_t)(val[4] | ((int16_t)val[5] << 8));

	point = point * 9.81 / 1000 * acc->sense;
	if (acc->offset != nullptr)
		point = point - acc->offset;

	acc->data.push_back(point);
}


void IMU::getGyro() {
	if (i2c->getDeviceAddress() != ADR_G)
		i2c->setDeviceAddress(ADR_G);

	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	i2c->readBlock(0x80 | GYR_OUT_X, val, 6);		// 0x80 needed for readBlock() function

	shared_ptr<geometry::CNPoint3D> point = make_shared<geometry::CNPoint3D>();

	// conversion from milli degree per second in [mdps] to degree per second [dps]
	point->x = (int16_t)(val[0] | ((int16_t)val[1] << 8));
	point->y = (int16_t)(val[2] | ((int16_t)val[3] << 8));
	point->z = (int16_t)(val[4] | ((int16_t)val[5] << 8));

	point = point / 1000 * gyr->sense;
	if (acc->offset != nullptr)
		point = point - gyr->offset;

	gyr->data.push_back(point);
}

void IMU::getMagnet() {
	if (i2c->getDeviceAddress() != ADR_XM)
		i2c->setDeviceAddress(ADR_XM);

	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	i2c->readBlock(0x80 | MAG_OUT_X, val, 6);		// 0x80 needed for readBlock() function

	shared_ptr<geometry::CNPoint3D> point = make_shared<geometry::CNPoint3D>();

	// conversion from milli gauss [mgauss] to radiant and degree
	point->x = (int16_t)(val[0] | ((int16_t)val[1] << 8));
	point->y = (int16_t)(val[2] | ((int16_t)val[3] << 8));
	point->z = (int16_t)(val[4] | ((int16_t)val[5] << 8));

	point = point * mag->sense;

	mag->data.push_back(point);
}

void IMU::getTemp() {
	if (i2c->getDeviceAddress() != ADR_XM)
		i2c->setDeviceAddress(ADR_XM);

	uint8_t val[2] = { 0x00, 0x00 };
	for(int i=0; i<2; i++)
		val[i] = i2c->readByte(TEMP_OUT + i);

	int16_t t = (int16_t)(val[0] | ((int16_t)val[1] << 8));

	// Fahreinheit -> Celsiuis
	temperature = (t - 32) / 1.8;
}

void IMU::getData(timeval time_now) {
	this->getAccel();
	this->getGyro();
	this->getMagnet();
	this->getTemp();

	last_updated = time_now;
}

msl_actuator_msgs::IMUData IMU::sendData(timeval time_now){
	acc->updateInternalValues();
	gyr->updateInternalValues();
	mag->updateInternalValues();


/*
	std::cout << "ACC X: " << acc->mean->x << std::endl;
	std::cout << "ACC Y: " << acc->mean->y << std::endl;
	std::cout << "ACC Z: " << acc->mean->z << std::endl;

	std::cout << "GYR X: " << gyr->mean->x << std::endl;
	std::cout << "GYR Y: " << gyr->mean->y << std::endl;
	std::cout << "GYR Z: " << gyr->mean->z << std::endl;

	std::cout << "Angle rad: " << mag->angle_rad << std::endl;
	std::cout << "Angle deg: " << mag->angle_deg << std::endl;

	std::cout << "TEMP: " << temperature << std::endl;
*/
	msl_actuator_msgs::IMUData msg;
	msg.acceleration.x = acc->mean->x;
	msg.acceleration.y = acc->mean->y;
	msg.acceleration.z = acc->mean->z;
	msg.accelSens = acc->sense;
	msg.gyro.x = gyr->mean->x;
	msg.gyro.y = gyr->mean->y;
	msg.gyro.z = gyr->mean->z;
	msg.gyroSens = gyr->sense;
	msg.magnet.x = mag->mean->x;
	msg.magnet.y = mag->mean->y;
	msg.magnet.z = mag->mean->z;
	msg.magnetSens = mag->sense;
	msg.temperature = temperature;
	msg.time = (unsigned long long)time_now.tv_sec*1000000 + time_now.tv_usec;

	last_sended = time_now;

	return msg;
}


