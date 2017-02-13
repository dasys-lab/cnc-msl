#pragma once

#include <cnc_geometry/CNPositionAllo.h>
#include <fstream>
#include <iostream>
#include <memory>

namespace msl
{

class Prediction
{
  public:
    Prediction();
    virtual ~Prediction();

    double rotationVelocity(int ms);
    double angle(int ms);
    std::unique_ptr<std::pair<std::shared_ptr<geometry::CNPositionAllo>, double>> angleAndPosition(int ms);
    void monitoring();

  private:
    std::ofstream debugAngle;
    std::ofstream debugAnglePosition;
    std::ofstream debugRotationVel;
    double MAX_ACCELERATION = 3.0;
    double maxRotationAccel;
    double magicNumber;
};

} /* namespace msl */
