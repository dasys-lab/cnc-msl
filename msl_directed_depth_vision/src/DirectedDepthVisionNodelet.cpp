/*
 * DirectedDepthVisionNodelet.cpp
 *
 *  Created on: 01.09.2015
 *      Author: tobi
 */

#include "DirectedDepthVisionNodelet.h"

//rosrun nodelet nodelet load msl_directed_depth_vision/DirectedDepthVisit /camera/camera_nodelet_manager __name:=directedDepthVision

namespace msl_vision
{
	DirectedDepthVisionNodelet::DirectedDepthVisionNodelet()
	{
		counter = 0;
		viewTime = 0;
		convTime = 0;
		voxelSize = 5; //in cm
	}

	DirectedDepthVisionNodelet::~DirectedDepthVisionNodelet()
	{
	}

	void DirectedDepthVisionNodelet::onInit()
	{
		nh = getNodeHandle();
		pclSub = nh.subscribe("/camera/depth/points", 10, &DirectedDepthVisionNodelet::PointCloudCallback,this);
		irImgSub = nh.subscribe("/camera/ir/image", 10, &DirectedDepthVisionNodelet::IRImageCallback,this);

		filteredPclPub = nh.advertise<sensor_msgs::PointCloud2>("/DirectedDepthVision/SegmentedPCL", 10);

		viewer = new pcl::visualization::CloudViewer("DirectedDepth PCL");

		cloud_in = boost::make_shared<pcl::PointCloud<PointTypeIO> >();
		cloud_in->points.resize(640 * 480);
		cloud_out = boost::make_shared<pcl::PointCloud<PointTypeIO> >();
		cloud_out->points.resize(640 * 480);

		cloud_xyz = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
		cloud_xyz->points.resize(640 * 480);

		cloud_with_normals = boost::make_shared<pcl::PointCloud<PointTypeFull> >();

		cloud_f = boost::make_shared<pcl::PointCloud<PointTypeIO> >();
	}

	void DirectedDepthVisionNodelet::IRImageCallback(sensor_msgs::ImagePtr msg) {

		irImage = msg;

		int max = -99999999;
		int min = 999999999;
		for(int i = 0; i < msg->data.size(); i++) {
//			cout << ((msg->data[i] << 8) + msg->data[i++])  << " ";
			int value = ((msg->data[i] << 8) + msg->data[i++]);
			if(max < value) {
				max = value;
			}
			if(min > value) {
				min = value;
			}

		}
//		cout << "min, max: " << min << " " << max << endl;
	}

	void DirectedDepthVisionNodelet::PointCloudCallback(sensor_msgs::PointCloud2Ptr msg)
	{

		if(irImage != NULL) {

			pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(new pcl::IndicesClusters),
					large_clusters(new pcl::IndicesClusters);
			pcl::search::KdTree<PointTypeIO>::Ptr search_tree(new pcl::search::KdTree<PointTypeIO>);
			pcl::console::TicToc tt;

//			pcl::fromROSMsg(*msg, *cloud_xyz)y;

//			msg->fields[3].name ="intensity";

			pcl::fromROSMsg(*msg, *cloud_in);

			int irCounter = 0;

//			cout << "xyz " << cloud_xyz->points.size() << endl;

//				cout << cloud_in->points.size() << " " << cloud_xyz->points.size() << endl;
			for(int i = 0; i < cloud_in->points.size(); i++) {
//				cout << i << " ";
//				cloud_in->points[i].x = cloud_xyz->points[i].x;
//				cloud_in->points[i].y = cloud_xyz->points[i].y;
//				cloud_in->points[i].z = cloud_xyz->points[i].z;
				double intensity = (irImage->data[irCounter] << 8) + irImage->data[irCounter++];
				cloud_in->points[i].intensity = intensity;
			}

//			viewer->showCloud(cloud_in);

			// Downsample the cloud using a Voxel Grid class
//			std::cerr << "Downsampling...\n", tt.tic();
			pcl::VoxelGrid<PointTypeIO> vg;
			vg.setInputCloud(cloud_in);
			vg.setLeafSize(voxelSize / 100, voxelSize / 100, voxelSize / 100);
			vg.setDownsampleAllData(true);
			vg.filter(*cloud_out);
//			std::cerr << ">> Done: " << tt.toc() << " ms, " << cloud_out->points.size() << " points\n";

//			std::cerr << "Computing normals...\n", tt.tic();
			pcl::copyPointCloud(*cloud_out, *cloud_with_normals);
			pcl::NormalEstimationOMP<PointTypeIO, PointTypeFull> ne; // multi core / multi thread
	//		pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne; // single core / single thread
			ne.setInputCloud(cloud_out);
			ne.setSearchMethod(search_tree);
			ne.setRadiusSearch((voxelSize / 100)*2); // neighbour radius for calculating normal
			ne.compute(*cloud_with_normals);
//			std::cerr << ">> Done: " << tt.toc() << " ms\n";

//			 Set up a Conditional Euclidean Clustering class
//			std::cerr << "Segmenting to clusters...\n", tt.tic();
			pcl::ConditionalEuclideanClustering<PointTypeFull> cec(true);
			cec.setInputCloud(cloud_with_normals);
			cec.setConditionFunction(&DirectedDepthVisionNodelet::enforceIntensitySimilarity);
			cec.setClusterTolerance((voxelSize / 100)*5);
			cec.setMinClusterSize(cloud_with_normals->points.size() / 1000);
			cec.setMaxClusterSize(cloud_with_normals->points.size());
			cec.segment(*clusters);
			cec.getRemovedClusters(small_clusters, large_clusters);
//			std::cerr << ">> Done: " << tt.toc() << " ms\n";

			int minInt = 99999999;
			int maxInt = -9999999;
			for(int i = 0; i < cloud_out->points.size(); i++) {
				if(minInt > cloud_out->points[i].intensity) {
					minInt = cloud_out->points[i].intensity;
				}
				if(maxInt < cloud_out->points[i].intensity) {
					maxInt = cloud_out->points[i].intensity;
				}
			}
			cout << "max - min: " << maxInt - minInt << endl;

			// Using the intensity channel for lazy visualization of the output
			for (int i = 0; i < small_clusters->size(); ++i) {
				for (int j = 0; j < (*small_clusters)[i].indices.size(); ++j)
					cloud_out->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
			}
	//		cout << "small cluster size: " << small_clusters->size() << endl;
			for (int i = 0; i < large_clusters->size(); ++i) {
				for (int j = 0; j < (*large_clusters)[i].indices.size(); ++j)
					cloud_out->points[(*large_clusters)[i].indices[j]].intensity = +10.0;
	//			cout << "large cluster size: " << (*large_clusters)[i].indices.size() << endl;
			}
			for (int i = 0; i < clusters->size(); ++i)
			{
				int label = rand() % 8;
				for (int j = 0; j < (*clusters)[i].indices.size(); ++j)
					cloud_out->points[(*clusters)[i].indices[j]].intensity = label;
			}

			viewer->showCloud(cloud_out);
		}

	}

	bool DirectedDepthVisionNodelet::enforceIntensitySimilarity(const PointTypeFull& point_a,
																const PointTypeFull& point_b, float squared_distance)
	{
		if (fabs(point_a.intensity - point_b.intensity) < 5.0f)
			return (true);
		else
			return (false);
	}

	bool DirectedDepthVisionNodelet::enforceCurvatureOrIntensitySimilarity(const PointTypeFull& point_a,
																			const PointTypeFull& point_b,
																			float squared_distance)
	{
		Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
		if (fabs(point_a.intensity - point_b.intensity) < 5.0f)
			return (true);
		if (fabs(point_a_normal.dot(point_b_normal)) < 0.05)
			return (true);
		return (false);
	}
	bool DirectedDepthVisionNodelet::customRegionGrowing(const PointTypeFull& point_a, const PointTypeFull& point_b,
															float squared_distance)
	{
		Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
		if (squared_distance < 10000)
		{
			if (fabs(point_a.intensity - point_b.intensity) < 8.0f)
				return (true);
			if (fabs(point_a_normal.dot(point_b_normal)) < 0.06)
				return (true);
		}
		else
		{
			if (fabs(point_a.intensity - point_b.intensity) < 3.0f)
				return (true);
		}
		return (false);
	}
} /* namespace msl_vision */

PLUGINLIB_EXPORT_CLASS(msl_vision::DirectedDepthVisionNodelet, nodelet::Nodelet)
