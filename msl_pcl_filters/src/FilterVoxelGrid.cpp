/*
 * FilterVoxelGrid.cpp
 *
 *  Created on: 29.10.2015
 *      Author: Tobias Schellien
 */

#include "msl_pcl_filters/FilterVoxelGrid.h"
namespace pcl_filters
{

	FilterVoxelGrid::FilterVoxelGrid()
	{
		vg = new pcl::VoxelGrid<PointTypeIO>();
	}

	FilterVoxelGrid::~FilterVoxelGrid()
	{
		delete vg;
	}

	void FilterVoxelGrid::processVoxelGridFilter(pcl::PointCloud<PointTypeIO>::Ptr cloud_in,
													pcl::PointCloud<PointTypeIO>::Ptr cloud_out, double voxelSize,
													bool downSample)
	{
		vg->setInputCloud(cloud_in);
		vg->setLeafSize(voxelSize, voxelSize, voxelSize);
		vg->setDownsampleAllData(downSample);
		vg->filter(*cloud_out);

	}


}
