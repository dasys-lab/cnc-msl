#include <ros/ros.h>
#include <memory>

namespace sensor_msgs
{
    ROS_DECLARE_MESSAGE(LaserScan)
}

namespace laserMotionCalibration
{
    class MotionCalibrationLaser
    {
    public:
        MotionCalibrationLaser(int argc, char **argv);
        ~MotionCalibrationLaser();
    private:
        void onScan(const sensor_msgs::LaserScanConstPtr& laserScan);
        void getThresholdGroups(const sensor_msgs::LaserScanConstPtr& laserScan, double threshold);

        ros::NodeHandle rosNode;
        ros::Subscriber subscriber;
        std::shared_ptr<ros::Publisher> publisher;
        ros::AsyncSpinner spinner;
    };
}
