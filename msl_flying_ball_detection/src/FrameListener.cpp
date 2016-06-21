/*
 * FrameListener.cpp
 *
 *  Created on: Jun 19, 2016
 *      Author: Stephan Opfer
 */

using namespace std;

#include "FrameListener.h"
#include <vector>
#include <OpenNI.h>
#include "OniSampleUtilities.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>

#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.7/pcl/console/parse.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.7/pcl/sample_consensus/sac.h>
#include <pcl-1.7/pcl/sample_consensus/ransac.h>
#include <pcl-1.7/pcl/sample_consensus/sac_model_sphere.h>

namespace msl
{

	FrameListener::FrameListener()
	{
		pubBall = this->rosNode.advertise<sensor_msgs::PointCloud>("/astra/ball", 10000);
		pub = this->rosNode.advertise<sensor_msgs::PointCloud>("/astra/depthCloud", 10000);
	}

	FrameListener::~FrameListener()
	{
	}

	void FrameListener::onNewFrame(openni::VideoStream& vidStream)
	{
		// initialize PointClouds
		pcl::PointCloud<pcl::PointXYZ>::Ptr ballCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud(new pcl::PointCloud<pcl::PointXYZ>);
//		pcl::PCDReader reader;
//		reader.read ("/home/emmeda/test_1.pcd", *rawCloud);

		rawCloud->width = vidStream.getVideoMode().getResolutionX();
		rawCloud->height = vidStream.getVideoMode().getResolutionY();
		rawCloud->is_dense = false;
		rawCloud->points.resize(rawCloud->width * rawCloud->height);

		// read depth image
		openni::VideoFrameRef depthFrame;
		vidStream.readFrame(&depthFrame);

		if (!depthFrame.isValid())
		{
			cout << "error! depthFrame is not valid!!!" << endl;
			return;
		}

		float pDepthHist[vidStream.getMaxPixelValue()];
		calculateHistogram(pDepthHist, vidStream.getMaxPixelValue(), depthFrame);

		int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);
		const openni::DepthPixel* depthPixel = (const openni::DepthPixel*) depthFrame.getData();
		int x, y = 0;

		for (int i = 0; i < rawCloud->points.size(); i++)
		{
			if (*depthPixel != 0)
			{
				x = i % rowSize;
				y = i / rowSize;
				float wX, wY, wZ;
				openni::CoordinateConverter::convertDepthToWorld(vidStream, y, x, *depthPixel, &wX, &wY, &wZ);

				rawCloud->points[i].x = wX/1000.0;
				rawCloud->points[i].y = wY/1000.0;
				rawCloud->points[i].z = wZ/1000.0;
			}
			depthPixel++;
		}


		// Version 1
		/*pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr sphere_model(
				new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(rawCloud));
		sphere_model->setRadiusLimits(0.12, 0.12);
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(sphere_model, 0.01);
		ransac.computeModel();
		std::vector<int> inliers;
		ransac.getInliers(inliers);

		// copies all inliers of the model computed to another PointCloud
		pcl::copyPointCloud<pcl::PointXYZ>(*rawCloud, inliers, *ballCloud);
		*/

		// Version 2
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_CIRCLE2D);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(1000);
		seg.setDistanceThreshold(0.005);
		seg.setRadiusLimits(0.1,0.15);

		seg.setInputCloud (rawCloud);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
		  std::cerr << "Could not estimate a sphere model for the given dataset." << std::endl;
		}
		else
		{
			pcl::copyPointCloud<pcl::PointXYZ>(*rawCloud, inliers->indices, *ballCloud);

			sensor_msgs::PointCloud pcl;
			vector<geometry_msgs::Point32> points;
			vector<sensor_msgs::ChannelFloat32> channel;
			sensor_msgs::ChannelFloat32 depthChannel;
			depthChannel.name = "depth";

			for (int i = 0; i < ballCloud->points.size(); i++)
			{
				geometry_msgs::Point32 p;
				p.x = rawCloud->points[i].x;
				p.y = rawCloud->points[i].y;
				p.z = rawCloud->points[i].z;
				points.push_back(p);
			}

			channel.push_back(depthChannel);
			pcl.channels = channel;
			pcl.points = points;
			pcl.header.frame_id = "camera_frame";
			pcl.header.stamp = ros::Time::now();
			this->pubBall.publish(pcl);
		}


		sensor_msgs::PointCloud pcl;
		vector<geometry_msgs::Point32> points;
		vector<sensor_msgs::ChannelFloat32> channel;
		sensor_msgs::ChannelFloat32 depthChannel;
		depthChannel.name = "depth";

		for (int i = 0; i < rawCloud->points.size(); i++)
		{
			geometry_msgs::Point32 p;
			p.x = rawCloud->points[i].x;
			p.y = rawCloud->points[i].y;
			p.z = rawCloud->points[i].z;
			points.push_back(p);
		}

		channel.push_back(depthChannel);
		pcl.channels = channel;
		pcl.points = points;
		pcl.header.frame_id = "camera_frame";
		pcl.header.stamp = ros::Time::now();
		this->pub.publish(pcl);
	}



} /* namespace msl */
