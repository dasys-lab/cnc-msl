///*
// * goal_detection.h
// *
// *  Created on: Jan 27, 2017
// *      Author:  Lisa Martmann
// */
//
//#ifndef INCLUDE_GOAL_DETECTION_H_
//#define INCLUDE_GOAL_DETECTION_H_
//
//#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "sensor_msgs/LaserScan.h"
//#include "geometry_msgs/PointStamped.h"
//#include "geometry_msgs/Point.h"
//#include "msl_msgs/PositionInfo.h"
//
//#include <iostream>
//#include <utility>
//#include <algorithm>
//#include <cmath>
//#include <tf/transform_datatypes.h>
//
//using namespace std;
//using namespace tf;
//using namespace sensor_msgs;
//using namespace msl_msgs;
//
//struct goal_detection_config {
//	double back_width;
//	double goal_depth;
//	double back_width_tolerance;
//	int maximums_angle_distance;
//	double view_area_angle;
//	double min_distance;
//	double max_distance;
//	double reduction_factor;
//};
//
//class goal_detection
//{
//	void scan_message_recieved(LaserScan::ConstPtr msg);
//	vector<double> reduce_points(vector<double> original, int factor);
//	vector<pair<int, double>> find_maximums(vector<double> points, int index_distance);
//	vector<Vector3> polar_to_cartesian(vector<pair<int, double>> polars, double start_angle,
//										double angle_increment);
//	vector<pair<Vector3, Vector3>> find_back_candidates(vector<Vector3> maximums, double back_width,
//														double back_width_tolerance);
//	vector<pair<int, double>> filter_points(vector<pair<int, double>> polars, double start_angle,
//											double angle_increment);
//	double calculate_angle(Vector3 a, Vector3 b);
//	bool satisfies_threshold(vector<int> vec, int value, int threshold);
//	void publish_message(ros::Publisher pub, double x, double y, double angle);
//	bool is_in_range(double compare_to, double value, double threshold);
//	bool is_in_range(int compare_to, int value, int threshold);
//	double rad_to_degree(double rad);
//	double degree_to_rad(double degree);
//	void print_help(goal_detection_config default_config);
//	bool parse_command_line_args(int argc, char *argv[], goal_detection_config &dest);
//
//
//	string scanner_topic = "/scan";
//	string publisher_topic = "/goal";
//	string node_name = "goal_detection";
//
////	Vector3 scanner_offset(0.2, 0.0, 0); // x-, y-, z-Offset, Scanner zu Torwartmitte
////
////	Vector3 z_axis(0, 0, 1);
////	Vector3 y_axis(0, 1, 0);
//
//};
//
//#endif /* INCLUDE_GOAL_DETECTION_H_ */
