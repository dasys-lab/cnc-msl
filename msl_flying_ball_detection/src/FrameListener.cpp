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

#include <pcl-1.7/pcl/console/parse.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/sample_consensus/sac.h>
#include <pcl-1.7/pcl/sample_consensus/ransac.h>
#include <pcl-1.7/pcl/sample_consensus/sac_model_sphere.h>

namespace msl
{

	FrameListener::FrameListener()
	{
		pub = this->rosNode.advertise<sensor_msgs::PointCloud>("/astra/depthCloud", 10000);
	}

	FrameListener::~FrameListener()
	{
	}

	void FrameListener::onNewFrame(openni::VideoStream& vidStream)
	{
		// initialize PointClouds
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

		float pDepthHist[vidStream.getMaxPixelValue()];
		openni::VideoFrameRef depthFrame;
		vidStream.readFrame(&depthFrame);

		if (depthFrame.isValid())
		{
			calculateHistogram(pDepthHist, vidStream.getMaxPixelValue(), depthFrame);
		}
		else
		{
			cout << "error! depthFrame is not valid!!!" << endl;
		}

		int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);
		sensor_msgs::PointCloud pcl;
		vector<geometry_msgs::Point32> points;
		vector<sensor_msgs::ChannelFloat32> channel;
		sensor_msgs::ChannelFloat32 depthChannel;
		depthChannel.name = "depth";

		const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame.getData();

		for (int y = 0; y < depthFrame.getHeight(); ++y)
		{
			const openni::DepthPixel* pDepth = pDepthRow;

			for (int x = 0; x < depthFrame.getWidth(); ++x, ++pDepth)
			{
				if (*pDepth != 0)
				{
					geometry_msgs::Point32 p;
					openni::CoordinateConverter::convertDepthToWorld(vidStream, y, x, *pDepth, &p.x, &p.y, &p.z);
					p.x = p.x / 1000.0;
					p.y = p.y / 1000.0;
					p.z = p.z / 1000.0;
					points.push_back(p);
					depthChannel.values.push_back(pDepthHist[*pDepth]);
				}
			}

			pDepthRow += rowSize;
		}

		channel.push_back(depthChannel);
		pcl.channels = channel;
		pcl.points = points;
		pcl.header.frame_id = "camera_frame";
		pcl.header.stamp = ros::Time::now();

		std::vector<int> inliers;

		// created RandomSampleConsensus object and compute the appropriated model
		pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));

		pcl::RandomSampleConsensus < pcl::PointXYZ > ransac(model_s,.01);
		//ransac.setDistanceThreshold(.01);
		ransac.computeModel();
		ransac.getInliers(inliers);

		// copies all inliers of the model computed to another PointCloud
		pcl::copyPointCloud < pcl::PointXYZ > (*cloud, inliers, *final);


		this->pub.publish(pcl);
	}

} /* namespace msl */
