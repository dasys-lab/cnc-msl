#include "LaserGoalDetection.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <visualization_msgs/Marker.h>
#include <limits>


using std::cout;
using std::endl;
using std::min;
using std::max;
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
    this->debugPublisher = std::make_shared<ros::Publisher>(rosNode.advertise<sensor_msgs::PointCloud>("/point_cloud", 10));
    this->lineCloudPublisher = std::make_shared<ros::Publisher>(rosNode.advertise<sensor_msgs::PointCloud>("/line_cloud", 10));
    this->linesPublisher = std::make_shared<ros::Publisher>(rosNode.advertise<visualization_msgs::Marker>("/lines", 10));
    this->pointPublisher = std::make_shared<ros::Publisher>(rosNode.advertise<visualization_msgs::Marker>("/points", 10));
    this->candPublisher = std::make_shared<ros::Publisher>(rosNode.advertise<visualization_msgs::Marker>("/candidates", 10));
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
    lineCloud->width = laserScan->ranges.size();
    // setting height to 1 because our pcl is unorganized
    cloud->height = 1;
    lineCloud->height = 1;

    for (int i = 150; i < (laserScan->ranges.size() - 150); i++)
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

    pclMsg.header.frame_id = "line_detection";
    pclMsg.points = points;
    debugPublisher->publish(pclMsg);

    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "line_detection";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.01;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    for (int i = 0; i < 5; i++)
    {
        auto inliers = this->detectLinePoints(cloud);

        auto lineVectorX = cloud->points.at(inliers.at(0)).x - cloud->points.at(inliers.at(inliers.size() - 1)).x;
        auto lineVectorY = cloud->points.at(inliers.at(0)).y - cloud->points.at(inliers.at(inliers.size() - 1)).y;
        auto length = sqrt(lineVectorX * lineVectorX + lineVectorY * lineVectorY);

        if ((!((length < 3.25 && length > 1.00) || (length < 0.6 && length > 0.15))) || inliers.size() < 30)
        {
            continue;
        }
        geometry_msgs::Point p1;
        p1.x = cloud->points.at(inliers.at(0)).x + lineVectorX * 0.2;
        p1.y = cloud->points.at(inliers.at(0)).y + lineVectorY * 0.2;
        p1.z = 0;

        geometry_msgs::Point p2;
        p2.x = cloud->points.at(inliers.at(inliers.size() - 1)).x - lineVectorX * 0.2;
        p2.y = cloud->points.at(inliers.at(inliers.size() - 1)).y - lineVectorY * 0.2;
        p2.z = 0;

        line_list.points.push_back(p1);
        line_list.points.push_back(p2);

//        cout << i << ": "  << length << endl;

        for ( auto j : inliers )
        {
            lineCloud->points.push_back(cloud->points.at(j));
        }
        this->deleteInliers(cloud, inliers);
    }
    linesPublisher->publish(line_list);



    visualization_msgs::Marker point_list;
    point_list.header.frame_id = "line_detection";
    point_list.header.stamp = ros::Time::now();
    point_list.ns = "points_and_lines";
    point_list.action = visualization_msgs::Marker::ADD;
    point_list.pose.orientation.w = 1.0;
    point_list.id = 3;
    point_list.type = visualization_msgs::Marker::POINTS;
    point_list.scale.x = 0.03;
    point_list.scale.y = 0.03;
    point_list.color.g = 1.0;
    point_list.color.a = 1.0;
    //point_list.lifetime = ros::Duration(0.05);
    //cout << "================ points size: "<< line_list.points.size() <<" ===================" << endl;

    if (line_list.points.size() == 0)
    	return;

    for (int i = 0; i < line_list.points.size() - 2; i += 2)
    {
    	//cout << "================= it:" << i << " ==================" << endl;

        for (int j = i + 2; j < line_list.points.size(); j += 2)
        {
        	if (i == j)
        		continue;

            auto line1Point1 = line_list.points.at(i);
            auto line1Point2 = line_list.points.at(i + 1);
            auto line2Point1 = line_list.points.at(j);
            auto line2Point2 = line_list.points.at(j + 1);

        	//cout << line1Point1 << ", " << line1Point2 << "  : " << line2Point1 << ", " << line2Point2 <<endl;

            float x12 = line1Point1.x - line1Point2.x;
            float x34 = line2Point1.x - line2Point2.x;
            float y12 = line1Point1.y - line1Point2.y;
            float y34 = line2Point1.y - line2Point2.y;

            auto delta = (x12*x34 + y12 * y34)/sqrt(x12*x12 + y12*y12)*sqrt(x34*x34 + y34*y34);

            if(abs(delta) > cos(5*M_PI/180))
            	continue;

            float c = x12 * y34 - y12 * x34;

            if (!(fabs(c) < 0.01))
            {
                float a = line1Point1.x * line1Point2.y - line1Point1.y * line1Point2.x;
                float b = line2Point1.x * line2Point2.y - line2Point1.y * line2Point2.x;
                float x = (a * x34 - b * x12) / c;
                float y = (a * y34 - b * y12) / c;

                if (!
                		 ((x <= max(line1Point1.x, line1Point2.x) && x >= min(line1Point1.x, line1Point2.x)
                		&& y <= max(line1Point1.y, line1Point2.y) && y >= min(line1Point1.y, line1Point2.y))

				 		&& (x <= max(line2Point1.x, line2Point2.x) && x >= min(line2Point1.x, line2Point2.x)
				         && y <= max(line2Point1.y, line2Point2.y) && y >= min(line2Point1.y, line2Point2.y)))
					)
                {
                			continue;
                }

                geometry_msgs::Point point;
                point.x = x;
                point.y = y;
                point.z = 0;
                point_list.points.push_back(point);

                cout << x << ", " << y << "  intersection: " << i/2 << " x " << j/2 << "  angle: " << delta <<endl;


            }
        }
    }

    visualization_msgs::Marker pointCandidates;
    pointCandidates.header.frame_id = "line_detection";
    pointCandidates.header.stamp = ros::Time::now();
    pointCandidates.ns = "points_and_lines";
    pointCandidates.action = visualization_msgs::Marker::ADD;
    pointCandidates.pose.orientation.w = 1.0;
    pointCandidates.id = 4;
    pointCandidates.type = visualization_msgs::Marker::LINE_LIST;
    pointCandidates.scale.x = 0.01;
    pointCandidates.color.b = 1.0;
    pointCandidates.color.a = 1.0;

    for (int i = 0; i < point_list.points.size() - 1; i++)
    {
        for (int j = i + 1; j < point_list.points.size(); j++)
        {
        	geometry_msgs::Point point1 = point_list.points.at(i);
        	geometry_msgs::Point point2 = point_list.points.at(j);

        	auto length = sqrt((point1.x-point2.x)*(point1.x-point2.x) + (point1.y-point2.y)*(point1.y-point2.y));

        	cout << length << endl;

        	if (length > 1.8 && length < 2.5) {
        		pointCandidates.points.push_back(point1);
        		pointCandidates.points.push_back(point2);
        	}
        }
    }

    pointPublisher->publish(point_list);
    candPublisher->publish(pointCandidates);

    for (auto pt : lineCloud->points)
    {
        geometry_msgs::Point32 point32;
        point32.x = pt.x;
        point32.y = pt.y;
        linePoints.push_back(point32);
    }
    lineMsg.header.frame_id = "line_detection";
    lineMsg.points = linePoints;
    lineCloudPublisher->publish(lineMsg);
}

vector<int> LaserGoalDetection::detectLinePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<int> inliers;
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(0.008);
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
