/*
 * sensor.cpp
 *
 *  Created on: Apr 27, 2016
 *      Author: Carpe Noctem
 */

#include "sensor.h"

Sensor::Sensor()
{
	this->angle_deg = 0;
	this->angle_rad = 0;
	this->sense = 0;
	this->temperature = 0;
	this->mean = make_shared<geometry::CNPoint3D>(0.0, 0.0, 0.0);
	this->offset = nullptr;
}

Sensor::~Sensor()
{
}

void Sensor::updateInternalValues()
{
	this->mean = geometry::calculateMean(this->data);
	if(this->mean != nullptr)
	{
		this->angle_rad = atan2(this->mean->y, this->mean->x);
		this->angle_deg = this->angle_rad * 180 / M_PI;
	}
	if(this->offset == nullptr)
	{
		this->offset = make_shared<geometry::CNPoint3D>(this->mean->x, this->mean->y, this->mean->z);
	}
	this->data.clear();
}
