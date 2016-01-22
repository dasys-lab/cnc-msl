/*
 * FloorCalibrator.h
 *
 *  Created on: 26.10.2015
 *      Author: tobi
 */

#ifndef CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_FLOORCALIBRATOR_H_
#define CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_FLOORCALIBRATOR_H_

#include <mrpt/utils/types.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/ransac.h>
#include <mrpt/math/ransac_applications.h>
#include <mrpt/math/geometry.h>
#include <fstream>
#include <iostream>
#include <SystemConfig.h>
#include <Configuration.h>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl-1.7/pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

namespace msl_vision
{

	class FloorCalibrator
	{
		public:
			FloorCalibrator();
			virtual ~FloorCalibrator();

			void calibrate(sensor_msgs::PointCloud2Ptr pcl);
		private:
			double min_planeratio;
			double dist_threshold;
			int min_inliers;
			string floorPlaneFilename;
			mrpt::math::CVectorFloat xs, ys, zs;

			supplementary::SystemConfig *sc;
			supplementary::Configuration *depthVision;

			void saveFloorPlane(size_t size, mrpt::math::TPlane* plane);
	};

} /* namespace msl_vision */

#endif /* CNC_MSLDRIVER_MSL_DIRECTED_DEPTH_VISION_SRC_FLOORCALIBRATOR_H_ */
