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

#include <msl_msgs/RefBoxCommand.h>
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
#include <pcl-1.7/pcl/sample_consensus/model_types.h>
#include <pcl-1.7/pcl/filters/voxel_grid.h>
#include <pcl-1.7/pcl/sample_consensus/method_types.h>
#include <pcl-1.7/pcl/kdtree/kdtree.h>

#include <pcl-1.7/pcl/ModelCoefficients.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/filters/voxel_grid.h>
#include <pcl-1.7/pcl/features/normal_3d.h>
#include <pcl-1.7/pcl/kdtree/kdtree.h>
#include <pcl-1.7/pcl/sample_consensus/method_types.h>
#include <pcl-1.7/pcl/sample_consensus/model_types.h>
#include <pcl-1.7/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.7/pcl/segmentation/extract_clusters.h>

#include "msl_actuator_msgs/KickControl.h"

//#define FLYBALL_DEBUG

namespace msl
{

	FrameListener::FrameListener() :
			flyingBallPositions(10), mayKick(false)
	{
		pubBall = this->rosNode.advertise<sensor_msgs::PointCloud>("/astra/ball", 10);
		pub = this->rosNode.advertise<sensor_msgs::PointCloud>("/astra/depthCloud", 10);
		this->kickControlPub = rosNode.advertise<msl_actuator_msgs::KickControl>("/KickControl", 10);
		refBoxCommandSub = this->rosNode.subscribe("/RefereeBoxInfoBody", 10, &FrameListener::onRefBoxCommand, (FrameListener*)this);
	}

	FrameListener::~FrameListener()
	{
	}

	void FrameListener::onNewFrame(openni::VideoStream& vidStream)
	{
		ros::Time start = ros::Time::now();
		unsigned long timeStamp = start.sec * 1000000000 + start.nsec;
		// initialize PointClouds
		pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud(new pcl::PointCloud<pcl::PointXYZ>);
		if (!this->fillCloudFromDepth(vidStream, rawCloud))
		{
			// vidStream depth image wasn't valid
			return;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointXYZ>);
		this->createVoxelCloud(rawCloud, 0.10f, voxelCloud);
#ifdef FLYBALL_DEBUG
		ros::Time end = ros::Time::now();
//		cout << "Voxel Time: " << end - start << endl;
#endif

		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(voxelCloud);

		int currentIdx;
		bool foundBall = false;
		vector<int> indices;
		vector<float> squareDistances;

		for (int y = 0; y < voxelCloud->height; y += 2)
		{
			for (int x = 0; x < voxelCloud->width; x += 2)
			{
				currentIdx = y * voxelCloud->width + x;
				tree->radiusSearch(voxelCloud->points[currentIdx], 0.5, indices, squareDistances, 30);
				if (indices.size() > 6 && indices.size() < 20)
				{
					// found matching ball cluster
					foundBall = true;
					break;
				}
			}
			if (foundBall)
				break;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr ballCloud(new pcl::PointCloud<pcl::PointXYZ>);
		if (!foundBall)
		{
#ifdef FLYBALL_DEBUG
			this->publishCloud(ballCloud, this->pubBall);
#endif
			return;
		}

		for (std::vector<int>::const_iterator pit = indices.begin(); pit != indices.end(); ++pit)
		{
			ballCloud->points.push_back(voxelCloud->points[*pit]);
		}

		ballCloud->width = ballCloud->points.size();
		ballCloud->height = 1;
		ballCloud->is_dense = true;

		Eigen::Vector4d centroid;
		pcl::compute3DCentroid(*ballCloud, centroid);

		for (int i = 0; i < ballCloud->points.size(); i++)
		{
			if (sqrt(
					(centroid.x() - ballCloud->points[i].x) * (centroid.x() - ballCloud->points[i].x)
							+ (centroid.y() - ballCloud->points[i].y) * (centroid.y() - ballCloud->points[i].y)
							+ (centroid.z() - ballCloud->points[i].z) * (centroid.z() - ballCloud->points[i].z)) > 0.16) // some point is further away from the centroid than 14cm
			{
				foundBall = false;
				break;
			}
		}

		if (foundBall)
		{
			//cout << "Ball is at: " << centroid.x() << ", " << centroid.y() << ", " << centroid.z() << endl;
			boost::shared_ptr<Eigen::Vector4d> opt = boost::make_shared<Eigen::Vector4d>(centroid);
			boost::shared_ptr<InformationElement<Eigen::Vector4d> > o = boost::make_shared<
					InformationElement<Eigen::Vector4d> >(opt, timeStamp);
			o->certainty = 1;
			flyingBallPositions.add(o);
			this->checkBallTrajectory(timeStamp);
		}
		else
		{
			ballCloud->clear();
		}

#ifdef FLYBALL_DEBUG
		ros::Time end2 = ros::Time::now();
//		cout << "Full Time: " << end2-start << endl;
		this->publishCloud(ballCloud, this->pubBall);
#endif
	}

	void FrameListener::onRefBoxCommand(msl_msgs::RefBoxCommandPtr msg)
	{
		if (msg->cmd ==msl_msgs::RefBoxCommand::START )
		{
			this->mayKick = true;
		}
		else
		{
			this->mayKick = false;
		}
	}

	void FrameListener::kick()
	{
#ifndef FLYBALL_DEBUG
		if (!mayKick)
		{
			return;
		}
#endif

		// fire upper extension
#ifdef FLYBALL_DEBUG
		cout << "----->>>> FIRE <<<< -----" << endl;
#endif
		msl_actuator_msgs::KickControl kc;
		kc.extension = msl_actuator_msgs::KickControl::UPPER_EXTENSION;
		kc.extTime = 1000;
		kc.enabled = true;
		kc.senderID = 0;
		kickControlPub.publish(kc);
	}

	void FrameListener::createVoxelCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leafSize,
											pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud)
	{
		// Create the filtering object
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(leafSize, leafSize, leafSize);
		sor.filter(*voxelCloud);
#ifdef FLYBALL_DEBUG
		this->publishCloud(voxelCloud, this->pub);
#endif
	}

	bool FrameListener::fillCloudFromDepth(openni::VideoStream& vidStream, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		cloud->width = vidStream.getVideoMode().getResolutionX();
		cloud->height = vidStream.getVideoMode().getResolutionY();
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);

		// read depth image
		openni::VideoFrameRef depthFrame;
		vidStream.readFrame(&depthFrame);

		if (!depthFrame.isValid())
		{
			cerr << "error! depthFrame is not valid!!!" << endl;
			return false;
		}

		// Fill the raw Cloud with the depth image from the Astra S Pro
		float pDepthHist[vidStream.getMaxPixelValue()];
		calculateHistogram(pDepthHist, vidStream.getMaxPixelValue(), depthFrame);

		int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);
		const openni::DepthPixel* depthPixel = (const openni::DepthPixel*)depthFrame.getData();
		int x, y = 0;

		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (*depthPixel != 0)
			{
				x = i % rowSize;
				y = i / rowSize;
				float wX, wY, wZ;
				openni::CoordinateConverter::convertDepthToWorld(vidStream, y, x, *depthPixel, &wX, &wY, &wZ);

				// depth image comes within [mm], so make it to [m]
				cloud->points[i].x = wX / 1000.0;
				cloud->points[i].y = wY / 1000.0;
				cloud->points[i].z = wZ / 1000.0;
			}
			depthPixel++;
		}

		return true;
	}

	void FrameListener::checkBallTrajectory(unsigned long time)
	{
		int hitCounter = 0;
		for (int i = 0; i < this->flyingBallPositions.getSize(); i++)
		{
			boost::shared_ptr<InformationElement<Eigen::Vector4d> > curBall = this->flyingBallPositions.getLast(i);
			boost::shared_ptr<InformationElement<Eigen::Vector4d> > lastBall = this->flyingBallPositions.getLast(i + 1);
			if (!curBall || !lastBall)
			{
				continue;
			}

			if (curBall->timeStamp < time - 500000000 || lastBall->timeStamp < time - 500000000) // older than 0.5 sec
			{
				break;
			}

			double diffZ = curBall->getInformation()->z() - lastBall->getInformation()->z();
			double diffY = curBall->getInformation()->y() - lastBall->getInformation()->y();

			double resultY = curBall->getInformation()->y() + diffY * lastBall->getInformation()->z() / diffZ;
			if (abs(resultY) < 0.55)
			{
				hitCounter++;
			}
		}

		if (hitCounter > 0)
		{
			this->kick();
		}
	}

	void FrameListener::publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud, ros::Publisher pub)
	{
		sensor_msgs::PointCloud pcl;
		vector<geometry_msgs::Point32> points;
		vector<sensor_msgs::ChannelFloat32> channel;
		sensor_msgs::ChannelFloat32 depthChannel;
		depthChannel.name = "depth";

		for (int i = 0; i < voxelCloud->points.size(); i++)
		{
			geometry_msgs::Point32 p;
			p.x = voxelCloud->points[i].x;
			p.y = voxelCloud->points[i].y;
			p.z = voxelCloud->points[i].z;
			points.push_back(p);
		}

		channel.push_back(depthChannel);
		pcl.channels = channel;
		pcl.points = points;
		pcl.header.frame_id = "camera_frame";
		pcl.header.stamp = ros::Time::now();
		pub.publish(pcl);
	}

} /* namespace msl */

