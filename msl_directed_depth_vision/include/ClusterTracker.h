/*
 * ClusterTracker.h
 *
 *  Created on: 02.11.2015
 *      Author: Tobias Schellien
 */

#ifndef CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_CLUSTERTRACKER_H_
#define CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_CLUSTERTRACKER_H_

#include <map>
#include <pcl-1.7/pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl-1.7/pcl/point_types.h>

#include "Cluster.h"

using namespace std;

namespace msl_vision
{
	class ClusterTracker
	{
		public:
			ClusterTracker();
			virtual ~ClusterTracker();
			void process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzseg, pcl::IndicesClustersPtr clusters);
			void includeCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzseg);
		private:
			std::vector<Cluster> clusterVec;
			std::vector<Cluster> newClusters;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visualize;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_indices;


	};
}
#endif /* CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_CLUSTERTRACKER_H_ */
