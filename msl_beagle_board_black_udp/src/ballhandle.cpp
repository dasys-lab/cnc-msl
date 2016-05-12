/*
 * ballhandle.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Lukas Will
 */


#include "ballhandle.h"

using namespace BlackLib;

	BallHandle::BallHandle(BeaglePWM::PwmPin pwm_name, const char *pin_names[]) {
		gpio = BeagleGPIO::getInstance();
		pwm = BeaglePWM::getInstance();
		pins = gpio->claim((char**) pin_names, 4);

		direction = left;
		direction_desired = left;
		speed = 0;
		speed_desired = 0;

		int outputIdxs[] = { dir, rst};
		pins->enableOutput(outputIdxs, 2);

		pins->clearBit(dir);
		pins->setBit(rst);

		pwm_pin = pwm_name;
		pwm->setPeriod(pwm_pin, period);
		pwm->setRunState(pwm_pin, true);
	}

	BallHandle::~BallHandle() {
		pwm->setRunState(pwm_pin, false);
		delete pwm;
		delete gpio;
	}

	void BallHandle::setBallHandling(int32_t value) {
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
			pwm->setDutyCycle(pwm_pin, speed);
			if (direction)
				pins->setBit(dir);
			else
				pins->clearBit(dir);
		}

		if (speed != speed_desired) {
			speed = speed_desired;
			pwm->setDutyCycle(pwm_pin, speed);
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


