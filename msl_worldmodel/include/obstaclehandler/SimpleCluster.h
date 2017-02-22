#pragma once

#include <memory>
#include <cnc_geometry/CNPointAllo.h>

namespace msl
{

class SimpleCluster
{
  public:
    SimpleCluster(std::shared_ptr<geometry::CNPointAllo> p); // TODO: allo or ego?
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
    bool checkAndMerge(std::shared_ptr<SimpleCluster> cluster, double varianceThreshold);
    double distanceTo(std::shared_ptr<SimpleCluster> cluster);
};

} /* namespace msl */
