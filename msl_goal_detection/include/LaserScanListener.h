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
#include <msl_msgs/Pose2dStamped.h>
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
		string scanner_topic;
		string publisher_topic;
		string filter_topic;
		ros::NodeHandle n;
		ros::Publisher publisher;
		//TODO check if obsolete
//		ros::Publisher publisher_filtered_max;
		ros::Subscriber sub;
		double back_width;
		double goal_depth;
		double back_width_tolerance;
		//distance between 2 laser scan indices
		double max_angle_distance;
		double view_area_angle;
		double min_distance;
		double max_distance;
		double reduction_factor;

		tf::Vector3 scanner_offset;
		tf::Vector3 z_axis;
		tf::Vector3 y_axis;

		string frame_id;

		void readConfigParameters();
		vector<double> reduce_points(sensor_msgs::LaserScanPtr msg);
		vector<pair<int, double>> find_maxima(sensor_msgs::LaserScanPtr msg);
//		vector<pair<int, double>> find_maxima(vector<double> reduced);
		bool satisfies_threshold(vector<int> vec, int value);
		bool is_in_range(double value);
		bool is_in_range(int compare_to, int value);
		vector<pair<int, double>> filter_points(vector<pair<int, double>> polars, sensor_msgs::LaserScanPtr msg);
		vector<tf::Vector3> polar_to_cartesian(vector<pair<int, double>> polars, sensor_msgs::LaserScanPtr msg);
		vector<pair<tf::Vector3, tf::Vector3>> find_back_candidates(vector<tf::Vector3> maximums);
		double calculate_angle(tf::Vector3 a, tf::Vector3 b);
		double rad_to_degree(double rad);
	};
}

#endif /* INCLUDE_LASERSCANLISTENER_H_ */
