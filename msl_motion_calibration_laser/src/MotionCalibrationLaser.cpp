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

		if (!initialized)
		{
			std::cout << "MIN ANGLE: " << LaserPoint::minAngle << " ANGLE ICNREMENT: " << LaserPoint::angleIncrement << std::endl;
			std::cout << "MIN ANGLE" << laserScan->angle_min << std::endl;
			std::cout << "MIN INCREMENT" << laserScan->angle_increment << std::endl;
			LaserPoint::setMinAngle(laserScan->angle_min);
			LaserPoint::setAngleIncrement(laserScan->angle_increment);
			std::cout << "MIN ANGLE: " << LaserPoint::minAngle << " ANGLE ICNREMENT: " << LaserPoint::angleIncrement << std::endl;
			initialized = true;
		}
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
#ifdef laserMotionCalibrationDebug
		cout << "intensityThreshold: " << intensityThreshold << endl;
		cout << "number of filtered points: " << filteredIntensities.size() << endl;
#endif

		const clock_t begin_time = clock();
		std::vector<std::shared_ptr<LaserPointGroup>> pointGroups;
		for (auto threshold : filteredIntensities)
		{
			pointGroups = this->getThresholdGroups(laserScan, threshold);
			auto numberOfGroups = pointGroups.size();
#ifdef laserMotionCalibrationDebug
			cout << "t=" << threshold << " => " << numberOfGroups << " groups" << endl;
#endif
			if (numberOfGroups == 2)
			{
				break;
			}
		}

#ifdef laserMotionCalibrationDebug
		geometry_msgs::PoseArray poses;
		for (auto pointGroup : pointGroups)
		{
			cout << pointGroup->getCenter() << "/" << pointGroup->getDistance() << "mm  ";
			for (auto point : pointGroup->points)
			{
				geometry_msgs::Point rosPoint;
				std::shared_ptr<geometry::CNPoint2D> coordinates = point.getXY();
				rosPoint.x = coordinates->x;
				rosPoint.y = coordinates->y;
				geometry_msgs::Pose pose;
				pose.position = rosPoint;
				poses.poses.push_back(pose);
			}
		}
		cout << endl;

		cout << clock() - begin_time << " µs" << endl;
#endif

		poses.header.frame_id = "laser"; // for debugging purposes
		publisher->publish(poses);
	}

	std::vector<std::shared_ptr<LaserPointGroup>> MotionCalibrationLaser::getThresholdGroups(const sensor_msgs::LaserScanConstPtr& laserScan, double threshold)
	{
		bool aboveThreshold = false;
		int numberOfGroups = 0;
		std::vector<std::shared_ptr<LaserPointGroup>> groups;
		std::shared_ptr<LaserPointGroup> currentGroup;

		int i = 0;
		int groupBegin = -1;
		for (auto intensity : laserScan->intensities)
		{
			if (!aboveThreshold && intensity > threshold)
			{
				aboveThreshold = true;
				numberOfGroups++;
				groupBegin = i;
				currentGroup = std::make_shared<LaserPointGroup>();
				groups.push_back(currentGroup);
				currentGroup->points.push_back(LaserPoint(i, intensity, laserScan->ranges[i]));
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
				currentGroup->points.push_back(LaserPoint(i, intensity, laserScan->ranges[i]));
			}

			i++;
		}

		return groups;
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
