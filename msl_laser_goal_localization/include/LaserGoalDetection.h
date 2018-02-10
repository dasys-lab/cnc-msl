#pragma once

#include <ros/spinner.h>
#include <ros/node_handle.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

namespace sensor_msgs
{
ROS_DECLARE_MESSAGE(LaserScan)
}

namespace msl
{
namespace goaldetection
{

class LaserGoalDetection
{
  public:
    LaserGoalDetection(int argc, char **argv);
    virtual ~LaserGoalDetection();

    void onScan(const sensor_msgs::LaserScanConstPtr& );

  private:
    ros::NodeHandle rosNode;
    ros::Subscriber subscriber;
    std::shared_ptr<ros::Publisher> debugPublisher;
    std::shared_ptr<ros::Publisher> linePublisher;
    ros::AsyncSpinner spinner;
};
}
} /* namespace msl */
