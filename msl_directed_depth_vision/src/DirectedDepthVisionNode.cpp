/*
 * DirectedDepthVisionNode.cpp
 *
 *  Created on: 28.10.2015
 *      Author: Tobias Schellien
 */

#include "ros/ros.h"

#include <mrpt/math/geometry.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "DirectedDepthVisionNode");

//	mrpt::math::CVectorDouble xs,ys,zs;
/*	points_map->getAllPoints(xs,ys,zs);
	ransac_detect_3D_planes(xs,ys,zs,out_detected_planes,threshold,min_inliers_for_valid_plane);*/

	ros::NodeHandle n;

	while(ros::ok()) {

		ros::spinOnce();
	}
	return 0;
}

