/*
 * sensor.h
 *
 *  Created on: Apr 27, 2016
 *      Author: Carpe Noctem
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_UDP_INCLUDE_SENSOR_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_UDP_INCLUDE_SENSOR_H_

#include "GeometryCalculator.h"
#include <math.h>

class Sensor
{
public:
	Sensor();
	virtual ~Sensor();

	void updateInternalValues();

	shared_ptr<geometry::CNPoint3D> mean;
	shared_ptr<geometry::CNPoint3D> offset;
	vector<shared_ptr<geometry::CNPoint3D>> data;
	float sense;
	float angle_rad;
	float angle_deg;
	float temperature;

private:

};

#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_UDP_INCLUDE_SENSOR_H_ */
