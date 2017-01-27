/*
 * LaserScanListener.h
 *
 *  Created on: Jan 27, 2017
 *      Author:  Lisa Martmann
 */

#ifndef INCLUDE_LASERSCANLISTENER_H_
#define INCLUDE_LASERSCANLISTENER_H_

#include <ros/ros.h>
#include <string>
#include "msl_msgs/PositionInfo.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_datatypes.h>

using namespace std;

namespace msl
{
	class LaserScanListener
	{
	public:
		LaserScanListener();
		virtual ~LaserScanListener();

		void onLaserScanReceived(sensor_msgs::LaserScanPtr msg);

	private:
		std::string scanner_topic = "/scan";
		std::string publisher_topic = "/goal";
		std::string filter_topic = "/goal/filter";
		ros::NodeHandle n;
		ros::Publisher publisher;
		ros::Publisher publisher_filtered_max;
		ros::Subscriber sub;
		double back_width;
		double goal_depth;
		double back_width_tolerance;
		double max_angle_distance;
		double view_area_angle;
		double min_distance;
		double max_distance;
		double reduction_factor;

		void readConfigParameters();
		vector<double> reduce_points(sensor_msgs::LaserScanPtr msg, double factor);
		vector<pair<int, double>> find_maximums(sensor_msgs::LaserScanPtr msg, double index_distance);
		bool satisfies_threshold(vector<int> vec, int value, int threshold);
		bool is_in_range(double compare_to, double value, double threshold);
		bool is_in_range(int compare_to, int value, int threshold);
		vector<pair<int, double>> filter_points(vector<pair<int, double>> polars, sensor_msgs::LaserScanPtr msg);
		vector<tf::Vector3> polar_to_cartesian(vector<pair<int, double>> polars, sensor_msgs::LaserScanPtr msg);
		vector<pair<tf::Vector3, tf::Vector3>> find_back_candidates(vector<tf::Vector3> maximums, double back_width,
																	double back_width_tolerance);
		double calculate_angle(tf::Vector3 a, tf::Vector3 b);
		double rad_to_degree(double rad);
	};
}

#endif /* INCLUDE_LASERSCANLISTENER_H_ */
