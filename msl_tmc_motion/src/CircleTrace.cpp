/*
 * CircleTrace.cpp
 *
 *  Created on: Aug 25, 2015
 *      Author: cnpaul
 */

#include "CircleTrace.h"
#include <SystemConfig.h>


namespace msl_driver
{
	double CircleTrace::TWO_PI = 2 * M_PI;
	CircleTrace::CircleTrace()
	{
		initialize();
	}

	CircleTrace::~CircleTrace()
	{
	}

	void CircleTrace::initialize()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->x = (*sc)["Motion"]->get<double>("Motion.CircleTrace.Initial X", NULL);
		this->y = (*sc)["Motion"]->get<double>("Motion.CircleTrace.Initial Y", NULL);
		this->angle = (*sc)["Motion"]->get<double>("Motion.CircleTrace.Initial Angle", NULL);

		this->movement[0] = this->angle;
		this->movement[1] = 0.0;
		this->movement[2] = 0.0;

		this->isStopwatchStarted = false;

	}

	void CircleTrace::trace(double movement[])
	{
		// Initialize angle, translation, and rotation with the internal state
		double angle = this->movement[0];
		double translation = this->movement[1];
		double rotation = this->movement[2];

		double xTemp = 0.0;
		double yTemp = 0.0;
		double radius = 0.0;

		if(angle > M_PI)
		{
			angle -= TWO_PI;
		}


		// Check if this is the first call
		if(!this->isStopwatchStarted)
		{
			this->isStopwatchStarted = true;
			gettimeofday(&t0, 0);
		}
		else
		{
			gettimeofday(&t1, 0);
			double delta = ((double)timedifference_msec(t0, t1)) / 1000;
			gettimeofday(&t0, 0);

			translation *= delta;
			rotation *= delta;

			if(rotation != 0)
			{
				// The radius is given by ((2 * Pi * r) / (2 * Pi)) * t
				radius = abs(translation / rotation);
				// Get the projection of the radius
				xTemp = abs(sin(rotation) * radius) * geometry::GeometryCalculator::sgn(translation);
				// The axis are rotated 90°!a
				yTemp = (radius - (cos(rotation)) * radius) * geometry::GeometryCalculator::sgn(rotation);
			}
			else
			{
				xTemp = translation;
				yTemp = 0;
			}

			double h =  this->angle + angle;
			if(h > M_PI)
			{
				h -= TWO_PI;
			}

			double cos_h = cos(h);
			double sin_h = sin(h);
			double xTemp1 = cos_h * xTemp - sin_h * yTemp;
			double yTemp1 = sin_h * xTemp - cos_h * yTemp;

			this->angle += rotation;

			while(this->angle > M_PI)
			{
				this->angle -= TWO_PI;
			}
			// Adjust angle if it's < 0
			while(this->angle < -M_PI)
			{
				this->angle += TWO_PI;
			}

			this->x = xTemp1;
			this->y = yTemp1;
		}

		this->movement[0] = movement[0];
		this->movement[1] = movement[1];
		this->movement[2] = movement[2];
	}

// TODO: Should be tested!
	float CircleTrace::timedifference_msec(struct timeval t0, struct timeval t1)
	{
	    return (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f;
	}

	double CircleTrace::getX()
	{
		return this->x;
	}
	double CircleTrace::getY()
	{
		return this->y;
	}
	double CircleTrace::getAngle()
	{
		return this->angle;
	}

	void CircleTrace::setX(double x)
	{
		this->x = x;
	}
	void CircleTrace::setY(double y)
	{
		this->y = y;
	}
	void CircleTrace::setAngle(double angle)
	{
		this->angle =  angle;
	}

} /* namespace msl_driver */
