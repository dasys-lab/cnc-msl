/*
 * DirectedDepthVisionNodelet.h
 *
 *  Created on: 01.09.2015
 *      Author: tobi
 */

#ifndef CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_DIRECTEDDEPTHVISIONNODELET_H_
#define CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_DIRECTEDDEPTHVISIONNODELET_H_


#include "msl_directed_depth_vision/FloorCalibrator.h"
#include "msl_directed_depth_vision/ClusterTracker.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <vector>
#include <string>
#include <SystemConfig.h>
#include <Configuration.h>
#include <pluginlib/class_list_macros.h>
#include "sensor_msgs/PointCloud2.h"

#include <msl_pcl_filters/FilterVoxelGrid.h>

#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/console/time.h>
#include <pcl-1.7/pcl/PCLPointCloud2.h>
#include <pcl-1.7/pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

namespace msl_vision
{

	class DirectedDepthVisionNodelet : public nodelet::Nodelet
	{
	public:
		DirectedDepthVisionNodelet();
		virtual ~DirectedDepthVisionNodelet();
		virtual void onInit();

	private:
		ros::NodeHandle nh;
		ros::Subscriber pclSub;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground; // only for visualize the plane

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp;

		supplementary::SystemConfig* sc;
		supplementary::Configuration* depthVision;

		pcl::visualization::CloudViewer *viewer;
		pcl_filters::FilterVoxelGrid *vg;
		pcl::console::TicToc tt;


		bool calibrateFloor = false;
		bool debugDisplay = false;
		string floorPlaneFilename;

		double voxelSize;
		double maxFloorDist;
		double a=0, b=0, c=0, d=0;

		double height = 480;
		double width = 640;

		FloorCalibrator floorCalib;
		ClusterTracker clusterTracker;

		void PointCloudCallback(sensor_msgs::PointCloud2Ptr msg);
		bool loadPlane();
		void projectPixelOnCenter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl);
		static bool customRegionGrowing(const pcl::PointXYZRGB& point_a, const pcl::PointXYZRGB& point_b, float squared_distance);

	};

} /* namespace msl_vision */

#endif /* CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_DIRECTEDDEPTHVISIONNODELET_H_ */
