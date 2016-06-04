/*
 * FloorCalibrator.cpp
 *
 *  Created on: 26.10.2015
 *      Author: tobi
 */

#include "FloorCalibrator.h"

namespace msl_vision
{

	FloorCalibrator::FloorCalibrator()
	{
		sc = supplementary::SystemConfig::getInstance();
		depthVision = (*sc)["DirectedDepthVision"];

		min_planeratio = depthVision->get<double>("DepthVision", "FloorCalibration", "MIN_PLANERATIO", NULL);
		dist_threshold = depthVision->get<double>("DepthVision", "FloorCalibration", "DIST_THRESHOLD", NULL);
		min_inliers = depthVision->get<int>("DepthVision", "FloorCalibration", "MIN_INLIERS", NULL);
		floorPlaneFilename = depthVision->get<string>("DepthVision", "FloorPlaneFile", "filename", NULL);
	}

	FloorCalibrator::~FloorCalibrator()
	{
	}

	void FloorCalibrator::calibrate(sensor_msgs::PointCloud2Ptr pcl) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_xyz(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::fromROSMsg(*pcl, *pcl_xyz);

		int height = pcl_xyz->height;
		int width = pcl_xyz->width;

//		for(int i = width/6; i < 5 * width/6; i++ ) {
//			for(int j = 2*height/3; j < height-10; j++) {
				for(int i = 0; i < width; i++ ) {
							for(int j = 0; j < height; j++) {
				double x = pcl_xyz->points[i+width*j].x;
				double y = pcl_xyz->points[i+width*j].y;
				double z = pcl_xyz->points[i+width*j].z;

				xs.push_back(x);
				ys.push_back(y);
				zs.push_back(z);
//				cout << pcl_xyz->points[i+width*j].x << " " << pcl_xyz->points[i+width*j].y << " " << pcl_xyz->points[i+width*j].z << endl;
			}
		}

		std::vector<std::pair<size_t,mrpt::math::TPlane> > detectedPlanes;
		if(xs.size() > 0 && ys.size() > 0 && zs.size() > 0) {
			cout << xs.size() << " " << ys.size() << " " << zs.size() << endl;
			mrpt::math::ransac_detect_3D_planes(xs, ys ,zs , detectedPlanes, dist_threshold, min_inliers);
		}
		int allInliers = 0;
		for(int i = 0; i < detectedPlanes.size(); i++) {
			allInliers = allInliers + detectedPlanes[i].first;
			ROS_INFO("Detected plane %d with %ld inliers.", i+1, detectedPlanes[i].first);
		}

		ROS_INFO("Sum of relevant inliers: %d", allInliers);

		if (detectedPlanes.size() > 0){

			double planeRatio = (double)detectedPlanes[0].first / allInliers;
			ROS_INFO("Floor has %.2f amount of the field of view.", planeRatio);

			if(planeRatio < min_planeratio) {
				ROS_ERROR("The calibration is not that accurate, be sure that there is some more free space infront of the robot!");
			} else {
				saveFloorPlane(detectedPlanes[0].first, &detectedPlanes[0].second);
			}
		} else {
			ROS_ERROR("Couldn't calibrate the floor, no plane detected!");
		}
	}

	void FloorCalibrator::saveFloorPlane(size_t size, mrpt::math::TPlane* plane) {
		double a = plane->coefs[0], b = plane->coefs[1], c =plane->coefs[2], d = plane->coefs[3];
		ROS_INFO("Detected floor plane: %f*x + %f*y + %f*z + %f = 0", a, b, c, d);
//		string dir = sc->getConfigPath() +  sc->getHostname() + "/" + floorPlaneFilename;
		string dir = sc->getConfigPath() +  floorPlaneFilename;
		ofstream outfile(dir);

		if(outfile.is_open()) {
			ROS_INFO("Saving floor plane to file: %s", dir.c_str());
			outfile << a << " " << b << " " << c << " " << d;
			outfile.close();
		} else {
			ROS_ERROR("Couldn't save the plane to directory: %s", dir.c_str());
		}
	}

} /* namespace msl_vision */
