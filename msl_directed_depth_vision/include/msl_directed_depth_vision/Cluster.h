/*
 * Cluster.h
 *
 *  Created on: 02.11.2015
 *      Author: Tobias Schellien
 */

#ifndef CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_CLUSTER_H_
#define CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_CLUSTER_H_

#include <pcl-1.7/pcl/segmentation/conditional_euclidean_clustering.h>
#include <SystemConfig.h>
#include <Configuration.h>

namespace msl_vision
{

	class Cluster
	{
	public:
		Cluster();
		virtual ~Cluster();
		void setCentroid(pcl::PointXYZRGB centroid);
		void setCluster(pcl::PointIndicesPtr cluster);
		void setCluster(__gnu_cxx::__alloc_traits<std::allocator<pcl::PointIndices> >::value_type&);

		double dist(pcl::PointXYZRGB a, pcl::PointXYZRGB b);
		pcl::PointXYZRGB* getCentroid();

		bool operator==(Cluster& c1);
		pcl::PointIndicesPtr iCluster;
		__gnu_cxx::__alloc_traits<std::allocator<pcl::PointIndices> >::value_type cluster;
		pcl::PointXYZRGB centroid;

	private:
		double maxDistBetwCentroid;
		supplementary::SystemConfig* sc;
		supplementary::Configuration* depthVision;
	};

} /* namespace msl_vision */

#endif /* CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_CLUSTER_H_ */
