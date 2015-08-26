/*
 * CircleTrace.h
 *
 *  Created on: Aug 25, 2015
 *      Author: cnpaul
 */

#ifndef CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_CIRCLETRACE_H_
#define CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_CIRCLETRACE_H_

#include <sys/time.h>
#include <math.h>
#include <cmath>
#include <GeometryCalculator.h>

namespace msl_driver
{

	class CircleTrace
	{
	public:
		CircleTrace();
		virtual ~CircleTrace();

		struct timeval t0;
		struct timeval t1;


		double getX();
		double getY();
		double getAngle();

		void setX(double x);
		void setY(double y);
		void setAngle(double angle);
		static double TWO_PI;

		void trace(double movement[]);

	protected:
		double x = 0.0;
		double y = 0.0;
		double angle = 0.0;
		double movement[];
		void initialize();
		float timedifference_msec(struct timeval t0, struct timeval t1);
		float elapsed;

		bool isStopwatchStarted = false;
	};

} /* namespace msl_driver */

#endif /* CNC_MSLDRIVER_MSL_TMC_MOTION_SRC_CIRCLETRACE_H_ */
