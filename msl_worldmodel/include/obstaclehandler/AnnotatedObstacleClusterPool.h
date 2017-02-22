#pragma once

#include <vector>

namespace msl
{
class AnnotatedObstacleCluster;
class AnnotatedObstacleClusterPool
{
  public:
    AnnotatedObstacleClusterPool();
    virtual ~AnnotatedObstacleClusterPool();
    void reset();
    int curIndex;
    const static int maxCount;
    std::vector<AnnotatedObstacleCluster *> daAOCs;
};

} /* namespace msl */
