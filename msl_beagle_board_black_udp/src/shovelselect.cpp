/*
 * shovelselect.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: Lukas Will
 */

#include "shovelselect.h"
#include <SystemConfig.h>

using namespace BlackLib;

	ShovelSelect::ShovelSelect(pwmName pwm_P) {
		pwm = new BlackPWM(pwm_P);

		pwm->setPeriodTime(period, nanosecond);
		pwm->setSpaceRatioTime(0, microsecond);
		pwm->setRunState(stop);

		auto sc = supplementary::SystemConfig::getInstance();
		this->kickPWM = (*sc)["bbb"]->get<int>("BBB.shovelKick", NULL);
		this->passPWM = (*sc)["bbb"]->get<int>("BBB.shovelPass", NULL);
		this->timeout = (*sc)["bbb"]->get<int>("BBB.timeout", NULL);

		enabled = false;
		init = false;
	}

	ShovelSelect::~ShovelSelect() {
		delete pwm;
	}

	bool ShovelSelect::checkTimeout(timeval time) {
		if ((TIMEDIFFMS(time, ping) > timeout) && enabled) {
			pwm->setRunState(stop);
			enabled = false;

			return true;
		}

		return false;
	}

	bool ShovelSelect::setShovel(bool passing, timeval time_now) {
		if (statePassing == passing && init) {
			return false;
		}

		init = true;
		ping = time_now;
		statePassing = passing;
		if (passing) {
			pwm->setSpaceRatioTime(passPWM, microsecond);
		} else {
			pwm->setSpaceRatioTime(kickPWM, microsecond);
		}
		if (!enabled) {
			pwm->setRunState(run);
			enabled = true;
		}

		return true;
	}

