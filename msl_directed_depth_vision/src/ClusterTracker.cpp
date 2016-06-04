/*
 * ClusterTracker.cpp
 *
 *  Created on: 02.11.2015
 *      Author: Tobias Schellien
 */

#include "ClusterTracker.h"

namespace msl_vision{

	ClusterTracker::ClusterTracker()
	{
	}

	ClusterTracker::~ClusterTracker()
	{
	}

	void ClusterTracker::includeCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster) {
		for(int i = 0; i < clusterVec.size(); i++) {
			 for (int j = 0; j < clusterVec[i].cluster.indices.size(); ++j) {
				int indice = clusterVec[i].cluster.indices[j];

				for(int k = 0; k < cloud_voxel->points.size(); k++) {
					if(cloud_voxel->points[k].x == cloud_cluster->points[indice].x &&
							cloud_voxel->points[k].y == cloud_cluster->points[indice].y &&
							cloud_voxel->points[k].z == cloud_cluster->points[indice].z) {

						cloud_voxel->points[k].r = clusterVec[i].centroid.r;
						cloud_voxel->points[k].g = clusterVec[i].centroid.g;
						cloud_voxel->points[k].b = clusterVec[i].centroid.b;
					}
				}
			}
		}
	}

	void ClusterTracker::process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, pcl::IndicesClustersPtr clusters) {

		newClusters.clear();

		for (int i = 0; i < clusters->size (); ++i) {
			Cluster c;
			pcl::PointXYZRGB centroid(rand() % 256, rand() % 256, rand() % 256);
			c.setCluster((*clusters)[i]);
			double x=0, y=0, z=0;

			for (int j = 0; j < (*clusters)[i].indices.size(); ++j) {
				int indice = (*clusters)[i].indices[j];
				x += cloud_cluster->points[indice].x;
				y += cloud_cluster->points[indice].y;
				z += cloud_cluster->points[indice].z;
			}
			centroid.x = x/ (*clusters)[i].indices.size();
			centroid.y = y/ (*clusters)[i].indices.size();
			centroid.z = z/ (*clusters)[i].indices.size();
			c.setCentroid(centroid);

			newClusters.push_back(c);
		}
		if(clusterVec.size() < 1 && newClusters.size() > 0) {
			clusterVec = newClusters;
		} else if(newClusters.size() > 0) {
			for(int  i = 0; i < newClusters.size(); i++) {
				for(int j = 0; j < clusterVec.size(); j++) {
					if(newClusters[i]== clusterVec[j]) {
						newClusters[i].getCentroid()->r = clusterVec[j].getCentroid()->r;
						newClusters[i].getCentroid()->g = clusterVec[j].getCentroid()->g;
						newClusters[i].getCentroid()->b = clusterVec[j].getCentroid()->b;
					}
				}
			}
			clusterVec = newClusters;
		}
	}
}

