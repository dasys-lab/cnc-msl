#include "laserMotionCalibration/MotionCalibrationLaser.h"
#include "ros/ros.h"
#include <memory>
#include <algorithm>
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/ChannelFloat32.h>
//#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <ctime>

const float INITIAL_INTENSITY_THRESHOLD = 0.95f;

namespace laserMotionCalibration
{
	using std::cout;
	using std::endl;
	using std::shared_ptr;
	using std::make_shared;

	MotionCalibrationLaser::MotionCalibrationLaser(int argc, char **argv) :
			spinner(4)
	{
		spinner.start();

		subscriber = rosNode.subscribe<sensor_msgs::LaserScan>("/scan", 10, &MotionCalibrationLaser::onScan,
																(MotionCalibrationLaser*)this);
		this->publisher = std::make_shared < ros::Publisher
				> (rosNode.advertise<geometry_msgs::PoseArray>("/intenseScan", 10));
	}

	MotionCalibrationLaser::~MotionCalibrationLaser()
	{
		spinner.stop();
	}

	void MotionCalibrationLaser::onScan(const sensor_msgs::LaserScanConstPtr& laserScan)
	{
		// choose a threshold based on max intensity

		auto intensities = laserScan->intensities;
		const float maxIntensity = *max_element(intensities.begin(), intensities.end());
		const int numOfIntensities = intensities.size();
		std::sort(intensities.begin(), intensities.end());

		const float intensityThreshold = intensities[INITIAL_INTENSITY_THRESHOLD * numOfIntensities];
		std::vector<float> filteredIntensities;
		for (auto threshold : intensities)
		{
			if (threshold > intensityThreshold)
			{
				filteredIntensities.push_back(threshold);
			}
		}
		cout << "intensityThreshold: " << intensityThreshold << endl;
		cout << "number of filtered points: " << filteredIntensities.size() << endl;

		const clock_t begin_time = clock();
		for (auto threshold : filteredIntensities)
		{
			this->getThresholdGroups(laserScan, threshold);
		}
		std::cout << clock() - begin_time << " µs" << std::endl;

		// cout << ":) " << ros::Time::now() << endl;

		// calculate XY positions - not actually needed

		geometry_msgs::PoseArray poses;

		int i = 0;
		for (auto intensity : laserScan->intensities)
		{
			if (intensity > intensityThreshold)
			{
				double range = laserScan->ranges.at(i);
				double angle = laserScan->angle_min + i * laserScan->angle_increment;

				geometry_msgs::Point point;
				point.x = cos(angle) * range;
				point.y = sin(angle) * range;

				geometry_msgs::Pose pose;
				pose.position = point;
				poses.poses.push_back(pose);
			}

			i++;
		}
		// do something

		poses.header.frame_id = "laser"; // for debugging purposes

		publisher->publish(poses);
	}

	void MotionCalibrationLaser::getThresholdGroups(const sensor_msgs::LaserScanConstPtr& laserScan, double threshold)
	{
		bool aboveThreshold = false;
		int numberOfGroups = 0;

		int i = 0;
		int groupBegin = -1;
		for (auto intensity : laserScan->intensities)
		{
			if (!aboveThreshold && intensity > threshold)
			{
				aboveThreshold = true;
				numberOfGroups++;
				groupBegin = i;
			}

			if (aboveThreshold && intensity < threshold)
			{
				aboveThreshold = false;
				int groupEnd = i - 1;
				//cout << "[" << numberOfGroups << "] " << groupEnd - groupBegin + 1 << " scans (" << groupBegin << "-" << groupEnd << ")" << endl;
			}

			if (aboveThreshold)
			{
				//cout << "[" << numberOfGroups << "] Scan " << i << " (intensity=" << intensity << ", distance=" << laserScan->ranges[i] << ")" << endl;
			}

			i++;
		}

		// cout << "choosing threshold=" << threshold << " yielded " << numberOfGroups << " groups" << endl << endl;
		// cout << threshold << ";" << numberOfGroups << endl;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_calibration_laser");
	laserMotionCalibration::MotionCalibrationLaser motionCalibrationLaser =
			laserMotionCalibration::MotionCalibrationLaser(argc, argv);

	while (ros::ok())
	{
		ros::Duration(0.5).sleep();
	}

	return 0;
}
