/*
 * ballhandle.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Lukas Will
 */


#include "ballhandle.h"

using namespace BlackLib;

	BallHandle::BallHandle(pwmName pwm_P, gpioName dir_P, gpioName reset_P, gpioName ff1_P, gpioName ff2_P) {
		pwm = new BlackPWM(pwm_P);
		dir = new BlackGPIO(dir_P, output, FastMode);
		reset = new BlackGPIO(reset_P, output, FastMode);
		ff1 = new BlackGPIO(ff1_P, input, FastMode);
		ff2 = new BlackGPIO(ff2_P, input, FastMode);

		// PWM Frequenz setzen pwm->setPeriodTime(5000, microsecond);
	}

	void BallHandle::setBallHandling(int8_t value) {
		// value > 0 -> left
		// value < 0 -> right
		if (value == 0) {
			pwm->setSpaceRatioTime(speed, microsecond);
			pwm->setRunState(stop);
			enabled = false;
		} else {
			if (!enabled) {
				pwm->setRunState(run);
				enabled = true;
			}
		}

		if ((value > 0) && (direction == static_cast<digitalValue>(right))) {
			direction_desired = static_cast<digitalValue>(left);
		}

		if ((value < 0) && (direction == static_cast<BlackLib::digitalValue>(left))) {
			direction_desired = static_cast<BlackLib::digitalValue>(right);
		}

		speed_desired = abs(value) * 10;
	}

	void BallHandle::checkTimeout() {
		if (enabled) {
			pwm->setRunState(stop);
			enabled = false;
		}
	}

	void BallHandle::controlBallHandling() {
		if (direction != direction_desired) {		// Direction Change, Slow Down Speed
			speed -= BallHandle_PWM_STEP_SIZE;
			if (speed < 0) {
				direction = direction_desired;
				speed = 0;
				pwm->setSpaceRatioTime(speed, microsecond);
				dir->setValue(direction);
			}
		} else {									// Keep Direction, Modify Speed
			if (speed_desired > speed) {
				speed += BallHandle_PWM_STEP_SIZE;
				if (speed > speed_desired)
					speed = speed_desired;
			} else {
				speed -= BallHandle_PWM_STEP_SIZE;
				if (speed < speed_desired)
					speed = speed_desired;
			}
			pwm->setSpaceRatioTime(speed, microsecond);
		}
	}

	uint8_t BallHandle::getError() {
		uint8_t ff = (ff1->getNumericValue() << 1) | ff2->getNumericValue();

		return ff;
	}


