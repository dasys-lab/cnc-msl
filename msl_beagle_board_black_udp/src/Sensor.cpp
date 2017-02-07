/*
 * sensor.cpp
 *
 *  Created on: Apr 27, 2016
 *      Author: Lukas Will
 */

#include "Sensor.h"

#include <cnc_geometry/Calculator.h>

Sensor::Sensor()
{
    this->angle_deg = 0;
    this->angle_rad = 0;
    this->sense = 0;
    this->temperature = 0;
}

Sensor::~Sensor()
{
}

void Sensor::updateInternalValues()
{
    this->mean = geometry::calculateMean(this->data);

    this->angle_rad = atan2(this->mean.y, this->mean.x);
    this->angle_deg = this->angle_rad * 180 / M_PI;

    this->offset = this->mean;
    this->data.clear();
}
