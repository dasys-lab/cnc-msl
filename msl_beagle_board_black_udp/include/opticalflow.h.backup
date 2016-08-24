/*
 * opticalflow.h
 *
 *  Created on: Mar 26, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_OPTICALFLOW_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_OPTICALFLOW_H_


#include "includes.h"


// ADNS3080 Registers
#define PRODUCT_ID 0x00
#define MOTION 0x02
#define DELTA_X 0x03
#define DELTA_Y 0x04
#define SQUAL 0x05
#define PIXEL_SUM 0x06
#define MAXIMUM_PIXEL 0x07
#define CONFIGURATION_BITS 0x0A
#define EXTENDEND_CONFIG 0x0B
#define DATA_OUT_LOWER 0x0C
#define DATA_OUT_UPPER 0x0D
#define SHUTTER_LOWER 0x0E
#define SHUTTER_UPPER 0x0F
#define FRAME_PERIOD_LOWER 0x10
#define FRAME_PERIOD_UPPER 0x11
#define MOTION_CLEAR 0x12
#define FRAME_CAPTURE 0x13
#define INVERSE_PRODUCT_ID 0x3F
#define PIXEL_BURST 0x40
#define MOTION_BURST 0x50

#define RESOLUTION 30


enum Pin
{
	ncs, npd, of_rst, led
};


class OpticalFlow {
	public:
		void		reset(void);
		uint8_t		read(uint8_t address);

					OpticalFlow(const char *pin_names[], BlackLib::BlackSPI *spi_P);
					~OpticalFlow();

		void 		adns_init(void);
		void		controlLED(bool enabled);
		void 		update_motion_burst();
		msl_actuator_msgs::MotionBurst getMotionBurstMsg();

		uint8_t 	getInverseProductId(void);
		uint8_t 	getProductId(void);


	private:
		BlackLib::BlackSPI *spi;
		BeagleGPIO *gpio;
		BeaglePins *pins;

		uint8_t 	img[1536];

		int16_t		x, y;
		int16_t 	qos, vQos;
		int8_t 		motionBurst[7];
		int8_t 		debugOF;

		timeval		last_updated, last_sended;


		void		write(uint8_t address, uint8_t value);

		uint8_t		getConfigurationBits(void);
		void		getFrame(uint8_t *image);
		void		getFrameBurst(uint8_t *image, uint16_t size);
		void 		getMotionBurst(int8_t *burst);

		void		setConfigurationBits(uint8_t conf);
};



#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_OPTICALFLOW_H_ */
