/*
 * ballhandle.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Lukas Will
 */


#include "ballhandle.h"

using namespace BlackLib;

	BallHandle::BallHandle(pwmName pwm_P, const char *pin_names[]) {
		pwm = new BlackPWM(pwm_P);

		gpio = BeagleGPIO::getInstance();
		pins = gpio->claim((char**) pin_names, 4);

		direction = left;
		direction_desired = left;

		int outputIdxs[] = { 0, 1};
		pins->enableOutput(outputIdxs, 2);

		pwm->setPeriodTime(period, nanosecond);
		pwm->setSpaceRatioTime(0, nanosecond);
		pwm->setRunState(run);

		pins->clearBit(dir);
		pins->setBit(rst);
	}

	BallHandle::~BallHandle() {
		delete pwm;
		delete gpio;
	}

	void BallHandle::setBallHandling(int8_t value) {
		// value > 0 -> left
		// value < 0 -> right
		if ((value > 0) && (direction == right)) {
			direction_desired = left;
		}

		if ((value < 0) && (direction == left)) {
			direction_desired = right;
		}

		// Check that value is in range from -period to period
		if (value > period) { value = period; }
		if (value < -period) { value = -period; }
		speed_desired = abs(value);

		gettimeofday(&last_ping, NULL);
	}

	void BallHandle::checkTimeout() {
		// Deactivates the BallHandling when controlBallHandling() is called next time
		timeval	t;
		gettimeofday(&t, NULL);
		if (TIMEDIFFMS(t, last_ping) > BallHandle_TIMEOUT) {
			this->setBallHandling(0);
		}
	}

	void BallHandle::controlBallHandling() {
		checkTimeout();
		if (direction != direction_desired && speed_desired != 0) {
			direction = direction_desired;
			speed = 0;
			pwm->setSpaceRatioTime(speed, nanosecond);		// Time for this Operation: 900us
			pins->setBit(direction);
			//dir->setValue(direction);						// Time for this Operation: 550us
		}

		if (speed != speed_desired) {
			speed = speed_desired;
			pwm->setSpaceRatioTime(speed, nanosecond);		// Time for this Operation: 900us
		}
	}

	Error BallHandle::getError() {
		int pin1 = pins->getBit(ff1);
		int pin2 = pins->getBit(ff2);

		if (pin1 < 0 || pin2 < 0)
			return programming;

		if (!pin1 && !pin2)
			return none;
		if (!pin1 && pin2)
			return bypass;
		if (pin1 && !pin2)
			return temperature;
		if (pin1 && pin2)
			return voltage;
	}


