/*
 * Cluster.cpp
 *
 *  Created on: 02.11.2015
 *      Author: Tobias Schellien
 */

#include "msl_directed_depth_vision/Cluster.h"

namespace msl_vision
{

	Cluster::Cluster()
	{
		sc = supplementary::SystemConfig::getInstance();
		depthVision = (*sc)["DirectedDepthVision"];

		maxDistBetwCentroid = depthVision->get<double>("DepthVision", "Cluster", "maxDistBetwCentroid", NULL);
	}

	Cluster::~Cluster()
	{
	}

	void Cluster::setCluster(pcl::PointIndicesPtr cluster) {
		this->iCluster = cluster;
	}
	void Cluster::setCluster(__gnu_cxx::__alloc_traits<std::allocator<pcl::PointIndices> >::value_type& cluster) {
		this->cluster = cluster;
	}
	void Cluster::setCentroid(pcl::PointXYZRGB centroid) {
		this->centroid = centroid;
	}
	pcl::PointXYZRGB* Cluster::getCentroid() {
		return &this->centroid;
	}

	double Cluster::dist(pcl::PointXYZRGB a, pcl::PointXYZRGB b) {
			return abs(sqrt((a.x - b.x) *  (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z)));
	}

	bool Cluster::operator == (msl_vision::Cluster& c1) {
		if(dist(*c1.getCentroid(), *this->getCentroid()) < maxDistBetwCentroid) {
			return true;
		} else {
			return false;
		}
	}

} /* namespace msl_vision */
