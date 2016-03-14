/*
 * lightbarrier.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: Lukas Will
 */


#include "lightbarrier.h"

using namespace BlackLib;

	LightBarrier::LightBarrier(adcName adc_P) {
		adc = new BlackADC(adc_P);
		threshold = 500;
	}

	LightBarrier::~LightBarrier() {
		delete adc;
	}

	bool LightBarrier::checkLightBarrier() {
//		if (adc->getNumericValue() > threshold) {
//			return true;
//		} else {
			return false;
//		}
	}

	bool LightBarrier::setTreshold(int th) {
		if ((th > 0) && (th < 1024)) {
			threshold = th;
			return true;
		} else {
			return false;
		}
	}
