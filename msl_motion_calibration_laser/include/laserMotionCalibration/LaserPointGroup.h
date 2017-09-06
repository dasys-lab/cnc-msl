/*
 * LaserPointGroup.h
 *
 *  Created on: Aug 23, 2017
 *      Author: cn
 */

#ifndef SRC_LASERPOINTGROUP_H_
#define SRC_LASERPOINTGROUP_H_

#include "LaserPoint.h"
#include <container/CNPoint2D.h>
#include <vector>
#include <SystemConfig.h>

namespace laserMotionCalibration
{
class LaserPointGroup
{
  public:
    LaserPointGroup();
    virtual ~LaserPointGroup();
    double getCenter();
    double getDistance();
    std::shared_ptr<geometry::CNPoint2D> getWeightedCenter();
    std::shared_ptr<geometry::CNPoint2D> getPillarCenter();
    std::vector<LaserPoint> points;

  private:
    static supplementary::SystemConfig *sc;
    static double pillarRadius;
    static double hokuyoCenterOffset;
};

} /* namespace laserMotionCalibration */

#endif /* SRC_LASERPOINTGROUP_H_ */
