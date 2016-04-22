/*
 * ballhandle.h
 *
 *  Created on: Mar 11, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_BALLHANDLE_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_BALLHANDLE_H_
#define TIMEDIFFMS(n,o) (((n).tv_sec-(o).tv_sec)*1000+((n).tv_usec-(o).tv_usec)/1000)
#define BallHandle_TIMEOUT 1000

#include <sys/time.h>

#include <BlackDef.h>
#include <BlackGPIO.h>
#include <BlackPWM.h>

#include <BeagleGPIO.h>
#include <BeaglePins.h>

enum Error
{
	none, bypass, temperature, voltage, programming
};

enum Direction
{
	left = 0, right = 1
};

enum BH_Pin
{
	dir, rst, ff1, ff2
};

class BallHandle
{
public:
	BallHandle(BlackLib::pwmName pwm_P, const char *pin_names[]);
	~BallHandle();

	void setBallHandling(int8_t value);
	void checkTimeout();
	void controlBallHandling();

	Error getError();

private:
	BlackLib::BlackPWM *pwm;
	BeagleGPIO *gpio;
	BeaglePins *pins;

	Direction direction;
	Direction direction_desired;

	bool enabled = false;
	const int period = 10000;
	int speed = 0, speed_desired = 0;
	timeval last_ping;

};

#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_BALLHANDLE_H_ */
