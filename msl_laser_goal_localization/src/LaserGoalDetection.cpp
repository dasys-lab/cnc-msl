#include "LaserGoalDetection.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

using std::cout;
using std::endl;
using std::make_shared;
using std::vector;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_goal_detection");
    msl::goaldetection::LaserGoalDetection laserGoalDetection = msl::goaldetection::LaserGoalDetection(argc, argv);

    while (ros::ok())
    {
        ros::Duration(0.5).sleep();
    }

    return 0;
}

namespace msl
{
namespace goaldetection
{

LaserGoalDetection::LaserGoalDetection(int argc, char **argv)
    : spinner(4)
{
    this->spinner.start();
    this->subscriber = rosNode.subscribe<sensor_msgs::LaserScan>("/scan", 10, &LaserGoalDetection::onScan, (LaserGoalDetection *)this);
    this->debugPublisher = std::make_shared<ros::Publisher>(rosNode.advertise<sensor_msgs::PointCloud>("/goal_cloud", 10));
    this->linePublisher = std::make_shared<ros::Publisher>(rosNode.advertise<sensor_msgs::PointCloud>("/line_cloud", 10));
}

LaserGoalDetection::~LaserGoalDetection()
{
    spinner.stop();
}

void LaserGoalDetection::onScan(const sensor_msgs::LaserScanConstPtr &laserScan)
{

    /*
     * some info for our hokuyo on foxy:
     * angle min: -2.35619
     * angle max:  2.35619
     * angle increment: 0.00436332
     * range min: 0.02
     * range max: 60
     */

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr lineCloud(new pcl::PointCloud<pcl::PointXYZ>());
    sensor_msgs::PointCloud pclMsg;
    sensor_msgs::PointCloud lineMsg;
    vector<geometry_msgs::Point32> points;
    vector<geometry_msgs::Point32> linePoints;

    cloud->width = laserScan->ranges.size();
    // setting height to 1 because our pcl is unorganized
    cloud->height = 1;
    lineCloud->width = laserScan->ranges.size();
    lineCloud->height = 1;

    for (int i = 0; i < laserScan->ranges.size(); i++)
    {
        double range = laserScan->ranges.at(i);
        double angle = laserScan->angle_min + i * laserScan->angle_increment;

        pcl::PointXYZ point;
        point.x = cos(angle) * range;
        point.y = sin(angle) * range;
        cloud->push_back(point);

        geometry_msgs::Point32 point32;
        point32.x = point.x;
        point32.y = point.y;
        points.push_back(point32);
    }

    pclMsg.header.frame_id = "goal_cloud";
    pclMsg.points = points;
    debugPublisher->publish(pclMsg);

    for (int i = 0; i < 6; i++)
    {

        auto inliers = this->detectLinePoints(cloud);

        for (auto j : inliers)
        {
            lineCloud->points.push_back(cloud->points.at(j));
        }
        this->deleteInliers(cloud, inliers);
    }

    for (auto pt : lineCloud->points)
    {
        geometry_msgs::Point32 point32;
        point32.x = pt.x;
        point32.y = pt.y;
        linePoints.push_back(point32);
    }

    lineMsg.header.frame_id = "line_cloud";
    lineMsg.points = linePoints;

    linePublisher->publish(lineMsg);

}

vector<int> LaserGoalDetection::detectLinePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    std::vector<int> inliers;
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(0.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    return inliers;
}

void LaserGoalDetection::deleteInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int> inliers)
{

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    pcl::PointIndices::Ptr inds(new pcl::PointIndices());
    for (auto i : inliers)
    {
        inds->indices.push_back(i);
    }

    extract.setNegative(true);
    extract.setIndices(inds);
    extract.filter(*cloud);
}

} /* namespace msl::goaldetection */
} /* namespace msl */
