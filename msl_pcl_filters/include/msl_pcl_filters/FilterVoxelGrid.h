/*
 * FilterVoxelGrid.h
 *
 *  Created on: 29.10.2015
 *      Author: Tobias Schellien
 */

#ifndef CNC_MSLDRIVER_MSL_PCL_FILTERS_SRC_FILTERVOXELGRID_H_
#define CNC_MSLDRIVER_MSL_PCL_FILTERS_SRC_FILTERVOXELGRID_H_

#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGB PointTypeIO;
namespace pcl_filters {
class FilterVoxelGrid
{
	private:
		pcl::VoxelGrid<PointTypeIO> *vg;
	public:
		FilterVoxelGrid();
		virtual ~FilterVoxelGrid();
		void processVoxelGridFilter(pcl::PointCloud<PointTypeIO>::Ptr cloud_in, pcl::PointCloud<PointTypeIO>::Ptr cloud_out, double voxelSize, bool downSample);
	};
}
#endif /* CNC_MSLDRIVER_MSL_PCL_FILTERS_SRC_FILTERVOXELGRID_H_ */
