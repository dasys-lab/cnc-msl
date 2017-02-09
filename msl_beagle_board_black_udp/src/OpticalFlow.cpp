/*
 * opticalflow.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: Lukas Will
 */

#include "opticalflow.h"

using namespace BlackLib;

OpticalFlow::OpticalFlow(const char *pin_names[], BlackSPI *spi_P) {
	ncs = new BlackGPIO(GPIO_112, output, FastMode);
	npd = new BlackGPIO(GPIO_117, output, FastMode);
	rst = new BlackGPIO(GPIO_115, output, FastMode);
	led = new BlackGPIO(GPIO_60, output, FastMode);
	spi = spi_P;


	x = 0;
	y = 0;
	qos = 0;
	vQos = 0;
	debugOF = 0;
}

OpticalFlow::~OpticalFlow() {
	delete ncs;
	delete npd;
	delete rst;
	delete led;
}

void OpticalFlow::adns_init(void) {
	//init of sensor

	ncs->setValue(high);
	reset();

	usleep(4000);

	setConfigurationBits(0x00);		// set resolution (0x00 = 400 counts per inch, 0x10 = 1600 cpi)
	led->setValue(high);
}

void OpticalFlow::controlLED(bool enabled) {
	led->setValue(static_cast<digitalValue>(enabled));
}

uint8_t OpticalFlow::read(uint8_t address) {
	ncs->setValue(low);
	spi->transfer(address, 75);		// wait t_SRAD

	uint8_t ret = spi->transfer(0x00);
	ncs->setValue(high);

	return ret;
}

void OpticalFlow::reset(void) {
	rst->setValue(high);
	rst->setValue(low);
	usleep(500);
}

void OpticalFlow::write(uint8_t address, uint8_t value) {
	ncs->setValue(low);
	spi->transfer(address | 0x80, 50);

	spi->transfer(value);
	ncs->setValue(high);
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

	ncs->setValue(low);
	spi->transfer(PIXEL_BURST);
	usleep(50);						// wait t_SRAD

	spi->transfer(write_arr, read_arr, size, 10);	// max. 1536 Pixels
	for(int i = 0; i < size; i++) {
		image[i] = (read_arr[i] << 2);
	}

	ncs->setValue(high);
}

uint8_t OpticalFlow::getInverseProductId(void) {
	return read(INVERSE_PRODUCT_ID);
}

void OpticalFlow::getMotionBurst(int8_t *burst) {
	uint8_t	write[7] = {0};
	uint8_t	read[7];

	ncs->setValue(low);

	spi->transfer(MOTION_BURST);
	usleep(75);

	// read all 7 bytes (Motion, Delta_X, Delta_Y, SQUAL, Shutter_Upper, Shutter_Lower, Maximum Pixels)
	spi->transfer(write, read, 7);
	for (int i = 0; i < 7; i++) {
		motionBurst[i] = read[i];
	}

	ncs->setValue(high);
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

