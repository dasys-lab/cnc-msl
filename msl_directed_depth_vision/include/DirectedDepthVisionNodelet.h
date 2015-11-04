/*
 * DirectedDepthVisionNodelet.h
 *
 *  Created on: 01.09.2015
 *      Author: tobi
 */

#ifndef CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_DIRECTEDDEPTHVISIONNODELET_H_
#define CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_DIRECTEDDEPTHVISIONNODELET_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <Eigen/StdVector>

#include <pluginlib/class_list_macros.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include <pcl-1.7/pcl/ModelCoefficients.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/console/time.h>
#include <pcl-1.7/pcl/features/normal_3d.h>
#include <pcl-1.7/pcl/features/normal_3d_omp.h>
#include <pcl-1.7/pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.7/pcl/PCLPointCloud2.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/visualization/point_cloud_color_handlers.h>
#include <pcl-1.7/pcl/visualization/point_cloud_handlers.h>
#include <pcl-1.7/pcl/search/kdtree.h>
#include <pcl-1.7/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.7/pcl/filters/voxel_grid.h>
#include <pcl/io/openni_grabber.h>
#include <pcl-1.7/pcl/io/openni_camera/openni_ir_image.h>

typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

using namespace std;

namespace msl_vision
{

	class DirectedDepthVisionNodelet : public nodelet::Nodelet
	{
	public:
		DirectedDepthVisionNodelet();
		virtual ~DirectedDepthVisionNodelet();
		virtual void onInit();

		static bool enforceIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance);
		static bool enforceCurvatureOrIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance);
		static bool customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance);

	private:
		ros::NodeHandle nh;
		ros::Subscriber pclSub;
		ros::Subscriber irImgSub;
		ros::Publisher filteredPclPub;

		pcl::PointCloud<PointTypeIO>::Ptr cloud_out;
		pcl::PointCloud<PointTypeIO>::Ptr cloud_in;
		pcl::PointCloud<PointTypeIO>::Ptr cloud_f;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;
		pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals;

		sensor_msgs::ImagePtr irImage;

		pcl::visualization::CloudViewer *viewer;
		pcl::console::TicToc tt;

		int counter;
		int viewTime;
		int convTime;
		double voxelSize;
		double clusterTolerance;
		void PointCloudCallback(sensor_msgs::PointCloud2Ptr msg);
		void IRImageCallback(sensor_msgs::ImagePtr msg);

	};

} /* namespace msl_vision */

#endif /* CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_DIRECTEDDEPTHVISIONNODELET_H_ */
