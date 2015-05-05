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

	//TODO: Wrong WhoAmI
	if(!this->whoami()) {
		std::cout << "WARNING: Wrong WhoAmI-response." << std::endl;
		// Was soll passieren?
	} else {
		std::cout << "Fine: WhoAmI" << std::endl;
	}

	this->init();
	std::cout << "Fine: Init" << std::endl;
}

IMU::~IMU() {

}

void IMU::init() {
	uint8_t val[2];

	// Enable Accel
	i2c->setDeviceAddress(ADR_XM);
	i2c->writeByte(CTRL_REG1_XM, 0x67);			// Accel Frequecy: 100Hz

	// Enable Magnet & Temp
	i2c->writeByte(CTRL_REG5_XM, 0xF4);			// Magnet Frequecy: 100Hz, & high resolution

	// Enable Gyro
	i2c->setDeviceAddress(ADR_G);
	i2c->writeByte(CTRL_REG1_G, 0x0F);			// Gyro

	this->setupAccel(ACC_AFS_4G);
	this->setupGyro(GYR_FS_2000DPS);
	this->setupMagnet(MAG_MDR_4GAUSS);
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

void IMU::setupAccel(uint8_t range) {
	uint8_t reg;

	i2c->setDeviceAddress(ADR_XM);
	reg = i2c->readByte(CTRL_REG2_XM);
	reg &= ~(0b00111000);

	i2c->writeByte(CTRL_REG2_XM, reg | range);
}

void IMU::setupGyro(uint8_t scale) {
	uint8_t reg;

	i2c->setDeviceAddress(ADR_G);
	reg = i2c->readByte(CTRL_REG4_G);
	reg &= ~(0b00110000);

	i2c->writeByte(CTRL_REG4_G, reg | scale);
}

void IMU::setupMagnet(uint8_t scale) {
	uint8_t reg;

	i2c->setDeviceAddress(ADR_XM);
	reg = i2c->readByte(CTRL_REG6_XM);
	reg &= ~(0b01100000);

	i2c->writeByte(CTRL_REG6_XM, reg | scale);
}

void IMU::getAccel() {
	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	i2c->setDeviceAddress(ADR_XM);
//	for(int i=0; i<6; i++) {
//		val[i] = i2c->readByte(ACCEL_OUT_X + i);
//	}
	std::cout << i2c->readBlock(ACCEL_OUT_X, val, sizeof(val)) << std::endl;

	accel.x = ((int16_t) val[1] << 8) | val[0];
	accel.y = ((int16_t) val[3] << 8) | val[2];
	accel.z = ((int16_t) val[5] << 8) | val[4];
}

void IMU::getGyro() {
	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	i2c->setDeviceAddress(ADR_G);
	for(int i=0; i<6; i++) {
		val[i] = i2c->readByte(GYRO_OUT_X + i);
	}

	gyro.x = ((int16_t) val[1] << 8) | val[0];
	gyro.y = ((int16_t) val[3] << 8) | val[2];
	gyro.z = ((int16_t) val[5] << 8) | val[4];
}

void IMU::getMagnet() {
	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	i2c->setDeviceAddress(ADR_XM);
	i2c->readBlock(MAGNET_OUT_X, val, sizeof(val));

	magnet.x = ((int16_t) val[1] << 8) | val[0];
	magnet.y = ((int16_t) val[3] << 8) | val[2];
	magnet.z = ((int16_t) val[5] << 8) | val[4];
}

void IMU::getTemp() {
	uint8_t val[2] = { 0x00, 0x00 };

	i2c->setDeviceAddress(ADR_XM);
	i2c->readBlock(TEMP_OUT, val, sizeof(val));

	temperature = ((int16_t) val[1] << 8) | val[0];
}

void IMU::updateData(timeval time_now) {
	if(TIMEDIFFMS(time_now, last_updated) > IMU_UPDATE_TIMEOUT) {
		this->getAccel();
		this->getGyro();
		this->getMagnet();
		this->getTemp();

		last_updated = time_now;

		std::cout << "Accel: " << accel.x << " - " << accel.y << " - " << accel.z << std::endl;
		//std::cout << "Gyro: " << gyro.x << " - " << gyro.y << " - " << gyro.z << std::endl;
		//std::cout << "Magnet: " << magnet.x << " - " << magnet.y << " - " << magnet.z << std::endl;
		//std::cout << "Temp: " << temperature << std::endl;


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

void IMU::testSensor() {
	uint8_t val[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	i2c->setDeviceAddress(ADR_XM);
	for(int i=0; i<6; i++) {
		val[i] = i2c->readByte(ACCEL_OUT_X + i);
	}

	accel.x = ((int16_t) val[1] << 8) | val[0];
	accel.y = ((int16_t) val[3] << 8) | val[2];
	accel.z = ((int16_t) val[5] << 8) | val[4];

	std::cout << "Accel - Byte: " << accel.x << " - " << accel.y << " - " << accel.z << std::endl;


	i2c->setDeviceAddress(ADR_XM);
	accel.x = i2c->readWord(ACCEL_OUT_X);
	accel.x = i2c->readWord(ACCEL_OUT_Y);
	accel.x = i2c->readWord(ACCEL_OUT_Z);


	std::cout << "Accel - Word: " << accel.x << " - " << accel.y << " - " << accel.z << std::endl;


	i2c->setDeviceAddress(ADR_XM);
	std::cout << "BlockSize: " << i2c->readBlock(ACCEL_OUT_X, val, sizeof(val)) << std::endl;

	accel.x = ((int16_t) val[1] << 8) | val[0];
	accel.y = ((int16_t) val[3] << 8) | val[2];
	accel.z = ((int16_t) val[5] << 8) | val[4];

	std::cout << "Accel - Block: " << accel.x << " - " << accel.y << " - " << accel.z << std::endl;
}


