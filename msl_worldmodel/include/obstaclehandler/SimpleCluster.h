/*
* SimpleCluster.h
 *
 *  Created on: Feb 11, 2016
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_OBSTACLEHANDLER_SIMPLECLUSTER_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_OBSTACLEHANDLER_SIMPLECLUSTER_H_

#include "GeometryCalculator.h"
#include <memory>

namespace msl
{

class SimpleCluster
{
  public:
    SimpleCluster(shared_ptr<geometry::CNPoint2D> p);
    virtual ~SimpleCluster();
    // FOR CLUSTERING
    int numObs;
    // Linear Sum
    int linearSumX;
    int linearSumY;
    // Square Sum
    int squareSum;
    int x;
    int y;
    double getVariance();
    bool checkAndMerge(shared_ptr<SimpleCluster> cluster, double varianceThreshold);
    double distanceTo(shared_ptr<SimpleCluster> cluster);
};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_OBSTACLEHANDLER_SIMPLECLUSTER_H_ */
