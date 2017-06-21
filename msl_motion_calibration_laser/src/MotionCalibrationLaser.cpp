#include "ros/ros.h"
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/ChannelFloat32.h>
//#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

void onScan(const sensor_msgs::LaserScanConstPtr& laserScan) {
    cout << "neuer scan: " << laserScan->header << endl;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "motion_calibration_laser");
        ros::NodeHandle rosNode;
        ros::AsyncSpinner spinner(4);
        spinner.start();

        ros::Subscriber subscriber = rosNode.subscribe<sensor_msgs::LaserScan>("/scan", 10, onScan);

        while (ros::ok())
        {
                ros::Duration(0.5).sleep();
        }

        spinner.stop();

        return 0;
}
