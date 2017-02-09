/*
 * Sensor.h
 *
 *  Created on: Apr 27, 2016
 *      Author: Lukas Will
 */

#pragma once

#include <cnc_geometry/CNPointAllo.h>


#include <math.h>
#include <memory>
#include <vector>

class Sensor
{
  public:
    Sensor();
    virtual ~Sensor();

    void updateInternalValues();

    geometry::CNPointAllo mean;
    std::shared_ptr<geometry::CNVecAllo> offset;
    std::vector<geometry::CNPointAllo> data;
    float sense;
    float angle_rad;
    float angle_deg;
    float temperature;
};
