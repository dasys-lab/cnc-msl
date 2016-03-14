/*
 * Prediction.h
 *
 *  Created on: Mar 11, 2016
 *      Author: sni
 */

#ifndef PREDICTION_H_
#define PREDICTION_H_

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
  std::unique_ptr<std::array<double,4>> angleAndPosition(int ms);
  void monitoring();

private:
  std::ofstream debugAngle;
  std::ofstream debugAnglePosition;
  std::ofstream debugRotationVel;
  double MAX_ACCELERATION = 3.0;
  double maxRotationAccel;
  double magicNumber;
};

} /* namespace ice */

#endif /* PREDICTION_H_ */
