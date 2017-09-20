#include <ros/ros.h>
#include <memory>
#include <laserMotionCalibration/LaserPointGroup.h>
#include <SystemConfig.h>
#include <container/CNPoint2D.h>

namespace sensor_msgs
{
    ROS_DECLARE_MESSAGE(LaserScan)
}

#define laserMotionCalibrationDebug

namespace laserMotionCalibration
{
    class MotionCalibrationLaser
    {
    public:
        MotionCalibrationLaser(int argc, char **argv);
        ~MotionCalibrationLaser();
      private:
        void onScan(const sensor_msgs::LaserScanConstPtr& laserScan);
        shared_ptr<geometry::CNPoint2D> calculateCoordinatesInPillarSystem(vector<std::shared_ptr<geometry::CNPoint2D>>& pillarCenters);
        std::vector<std::shared_ptr<LaserPointGroup>> getThresholdGroups(const sensor_msgs::LaserScanConstPtr& laserScan, double threshold);
        bool initialized = false;
        ros::NodeHandle rosNode;
        ros::Subscriber subscriber;
        std::shared_ptr<ros::Publisher> filteredPublisher;
        std::shared_ptr<ros::Publisher> resultsPublisher;
        std::shared_ptr<ros::Publisher> positionPublisher;
        ros::AsyncSpinner spinner;
        static supplementary::SystemConfig* sc;
        static double initialIntensityThreshold;
    };
}
