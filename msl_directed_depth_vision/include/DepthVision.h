/*
 * DepthVision.h
 *
 *  Created on: 04.06.2016
 *      Author: Tobias Schellien
 */

#ifndef CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_DEPTHVISION_H_
#define CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_DEPTHVISION_H_

#include "sensor_msgs/PointCloud.h"
#include "ros/ros.h"

#include "FloorCalibrator.h"
#include "ClusterTracker.h"

#include <nodelet/nodelet.h>
#include <vector>
#include <string>
#include <SystemConfig.h>
#include <Configuration.h>
#include <pluginlib/class_list_macros.h>

#include <msl_pcl_filters/FilterVoxelGrid.h>

#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/console/time.h>
#include <pcl-1.7/pcl/PCLPointCloud2.h>
#include <pcl-1.7/pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>


class DepthVision {
public:
	DepthVision();
	virtual ~DepthVision();

	void pointCloudCallback(sensor_msgs::PointCloud& pcl);
};

#endif /* CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_DEPTHVISION_H_ */
