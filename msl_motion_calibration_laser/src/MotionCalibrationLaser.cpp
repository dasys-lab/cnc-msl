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

const float INTENSITY_THRESHOLD = 0.7f;

using namespace std;
shared_ptr<ros::Publisher> publisher;

void onScan(const sensor_msgs::LaserScanConstPtr& laserScan)
{
    // choose a threshold based on max intensity

    const float maxIntensity = *max_element(laserScan->intensities.begin(), laserScan->intensities.end());
    const float intensityThreshold = INTENSITY_THRESHOLD * maxIntensity;

    // calculate XY positions

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

    poses.header.frame_id = "my_frame"; // for debugging purposes

    publisher->publish(poses);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_calibration_laser");
    ros::NodeHandle rosNode;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Subscriber subscriber = rosNode.subscribe<sensor_msgs::LaserScan>("/scan", 10, onScan);
    publisher = make_shared<ros::Publisher>(rosNode.advertise<geometry_msgs::PoseArray>("/intenseScan", 10));

    while (ros::ok())
    {
        ros::Duration(0.5).sleep();
    }

    spinner.stop();

    return 0;
}
