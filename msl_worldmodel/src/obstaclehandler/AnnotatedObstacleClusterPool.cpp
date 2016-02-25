/*
 * AnnotatedObstacleClusterPool.cpp
 *
 *  Created on: Feb 25, 2016
 *      Author: Stefan Jakob
 */

#include <obstaclehandler/AnnotatedObstacleClusterPool.h>
#include <obstaclehandler/AnnotatedObstacleCluster.h>

namespace msl
{

	const int AnnotatedObstacleClusterPool::maxCount = 1000;

	AnnotatedObstacleClusterPool::AnnotatedObstacleClusterPool() :
		daAOCs(maxCount), curIndex(0)
	{
		for (int i = 0; i < maxCount; i++)
		{
			daAOCs[i] = new AnnotatedObstacleCluster();
		}
	}

	AnnotatedObstacleClusterPool::~AnnotatedObstacleClusterPool()
	{
		for (int i = 0; i < maxCount; i++)
		{
			delete daAOCs[i];
		}
	}

} /* namespace msl */
