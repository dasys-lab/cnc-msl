/*
 * opticalflow.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: Lukas Will
 */

#include "opticalflow.h"

using namespace BlackLib;

OpticalFlow::OpticalFlow(const char *pin_names[], BlackSPI *spi_P) {
	spi = spi_P;

	gpio = BeagleGPIO::getInstance();
	pins = gpio->claim((char**) pin_names, 4);

	int outputIdxs[] = { ncs, npd, of_rst, led };
	pins->enableOutput(outputIdxs, 4);

	x = 0;
	y = 0;
	qos = 0;
	vQos = 0;
	debugOF = 0;
}

OpticalFlow::~OpticalFlow() {
	delete gpio;
}

void OpticalFlow::adns_init(void) {
	//init of sensor

	pins->setBit(ncs);
	reset();

	usleep(4000);

	setConfigurationBits(0x00);		// set resolution (0x00 = 400 counts per inch, 0x10 = 1600 cpi)
	pins->setBit(led);
}

void OpticalFlow::controlLED(bool enabled) {
	if (enabled)
		pins->setBit(led);
	else
		pins->clearBit(led);
}

uint8_t OpticalFlow::read(uint8_t address) {
	pins->clearBit(ncs);
	spi->transfer(address, 75);		// wait t_SRAD

	uint8_t ret = spi->transfer(0x00);
	pins->setBit(ncs);

	return ret;
}

void OpticalFlow::reset(void) {
	pins->setBit(of_rst);
	usleep(1);
	pins->clearBit(of_rst);
	usleep(10);
}

void OpticalFlow::write(uint8_t address, uint8_t value) {
	pins->clearBit(ncs);
	spi->transfer(address | 0x80, 50);

	spi->transfer(value);
	pins->setBit(ncs);
}

uint8_t OpticalFlow::getConfigurationBits(void) {
	return read(CONFIGURATION_BITS);
}

void OpticalFlow::getFrame(uint8_t *image) {
	reset();

	uint8_t regValue;
	bool isFirstPixel = false;

	write(FRAME_CAPTURE, 0x83);

	// wait 3 frame periods + 10 us for frame to be captured
	// min frame speed is 2000 frames/second so 1 frame = 500 us.  so 500 x 3 + 10 = 1510
	usleep(1510);

	for(int i=0; i<RESOLUTION;) {
		for(int j=0; j<RESOLUTION;) {
			regValue = read(FRAME_CAPTURE);
			if( !isFirstPixel && (regValue & 0x40) == 0 ) {
				i=0;
				j=0;
				break;
			} else {
				isFirstPixel = true;
				//pixelValue = ( regValue << 2);
				*image = ( regValue << 2);
				//*image = regValue;
				image++;	// next array cell address
			}
			usleep(50);
			j++;
		}
		if( isFirstPixel ) {
			i++;
		}
	}

	reset();
}

void OpticalFlow::getFrameBurst(uint8_t *image, uint16_t size) {
	uint8_t	write_arr[size];
	uint8_t	read_arr[size];

	write(FRAME_CAPTURE, 0x83);

	// wait 3 frame periods + 10 us for frame to be captured
	// min frame speed is 2000 frames/second so 1 frame = 500 us.  so 500 x 3 + 10 = 1510
	usleep(1510);					// wait t_CAPTURE

	pins->clearBit(ncs);
	spi->transfer(PIXEL_BURST);
	usleep(50);						// wait t_SRAD

	spi->transfer(write_arr, read_arr, size, 10);	// max. 1536 Pixels
	for(int i = 0; i < size; i++) {
		image[i] = (read_arr[i] << 2);
	}

	pins->setBit(ncs);
}

uint8_t OpticalFlow::getInverseProductId(void) {
	return read(INVERSE_PRODUCT_ID);
}

void OpticalFlow::getMotionBurst(int8_t *burst) {
	uint8_t	write[7] = {0};
	uint8_t	read[7];

	pins->clearBit(ncs);

	spi->transfer(MOTION_BURST);
	usleep(75);

	// read all 7 bytes (Motion, Delta_X, Delta_Y, SQUAL, Shutter_Upper, Shutter_Lower, Maximum Pixels)
	spi->transfer(write, read, 7);
	for (int i = 0; i < 7; i++) {
		motionBurst[i] = read[i];
	}

	pins->setBit(ncs);
}

uint8_t OpticalFlow::getProductId(void) {
	return read(PRODUCT_ID);
}

void OpticalFlow::setConfigurationBits(uint8_t conf) {
	write(CONFIGURATION_BITS, conf);
}

void OpticalFlow::update_motion_burst() {
	getMotionBurst(motionBurst);
	x += motionBurst[1];
	y += motionBurst[2];
	qos += motionBurst[3];

	if( x!=0 || y!=0 ) {
		vQos++;
	}
}


msl_actuator_msgs::MotionBurst OpticalFlow::getMotionBurstMsg() {
	msl_actuator_msgs::MotionBurst msg;

	int16_t tqos = 0;
	if( vQos != 0 ) {
		tqos = qos/vQos;
	}

	msg.x = x;
	msg.y = y;
	msg.qos = tqos;

	x = 0;
	y = 0;
	qos = 0;
	vQos = 0;

	return msg;
}

