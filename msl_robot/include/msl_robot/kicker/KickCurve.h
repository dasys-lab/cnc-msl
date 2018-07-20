/*
 * KickCurve.h
 *
 *  Created on: Jul 13, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_KICKCURVE_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_KICKCURVE_H_

using namespace std;

#include <string>
#include <vector>

namespace msl
{

class KickCurve
{
  public:
    KickCurve(string section);
    virtual ~KickCurve();
    int getKickPower(double dist, double velocity);

  protected:
    vector<double> kickCurvePower;
    vector<double> kickCurveDistance;
    double velocityReduction;
};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_KICKCURVE_H_ */
