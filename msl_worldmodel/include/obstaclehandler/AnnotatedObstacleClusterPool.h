/*
 * AnnotatedObstacleClusterPool.h
 *
 *  Created on: Feb 25, 2016
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_OBSTACLEHANDLER_ANNOTATEDOBSTACLECLUSTERPOOL_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_OBSTACLEHANDLER_ANNOTATEDOBSTACLECLUSTERPOOL_H_

using namespace std;

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
    vector<AnnotatedObstacleCluster *> daAOCs;
};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_OBSTACLEHANDLER_ANNOTATEDOBSTACLECLUSTERPOOL_H_ */
