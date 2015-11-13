/*
 * MotorConfig.h
 *
 *  Created on: Nov 11, 2015
 *      Author: emmeda
 */

#ifndef INCLUDE_MOTORCONFIG_H_
#define INCLUDE_MOTORCONFIG_H_

namespace msl_driver
{
	struct MotorConfig
	{
		int resolution;
		int maxSpeed;
		int nomSpeed;
		//public int delta;
		int denominator;
		int numerator;
		int windingTime;
		int maxCurrent;
		int nomCurrent;
		int chassisTime;

		int limitedCurrent;
		int wheelRadius;
		int gearReduction;
		double vmax;

		double pidKp;
		double pidB;
		double pidKi;
		double pidKd;
		double pidKdi;
		double linFactor;
		double smoothFactor;
		int maxErrorInt;
		double rotationErrorWeight;
		double rotationErrorByVeloWeight;
		double rotationErrorByAccelWeight;

		bool controlCurrent;
		int currentErrorBound;
		double currentKp;
		double currentKi;
		double currentKd;

		int deadBand;

		/*double maxAcceleration;
		double maxDecceleration;
		double maxRotForce;*/

		int accelBoundMin;
		int accelBoundMax;
		double rotationAccelBound;

		int failSafeRPMBound;
		int failSafePWMBound;
		int failSafeCycles;
	};
}

#endif /* INCLUDE_MOTORCONFIG_H_ */
