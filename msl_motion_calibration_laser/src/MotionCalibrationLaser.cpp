#include "laserMotionCalibration/MotionCalibrationLaser.h"
#include "ros/ros.h"
#include <algorithm>
#include <cmath>
#include <container/CNPosition.h>
#include <ctime>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <sensor_msgs/LaserScan.h>

namespace laserMotionCalibration
{
using std::cout;
using std::endl;
using std::shared_ptr;
using std::make_shared;

double MotionCalibrationLaser::initialIntensityThreshold = -1.0;
supplementary::SystemConfig *MotionCalibrationLaser::sc = supplementary::SystemConfig::getInstance();

MotionCalibrationLaser::MotionCalibrationLaser(int argc, char **argv)
    : spinner(4)
{
    spinner.start();

    subscriber = rosNode.subscribe<sensor_msgs::LaserScan>("/scan", 10, &MotionCalibrationLaser::onScan, (MotionCalibrationLaser *)this);
    this->filteredPublisher = std::make_shared<ros::Publisher>(rosNode.advertise<geometry_msgs::PoseArray>("/filteredScan", 10));
    this->resultsPublisher = std::make_shared<ros::Publisher>(rosNode.advertise<geometry_msgs::PoseArray>("/laserResults", 10));
    this->positionPublisher = std::make_shared<ros::Publisher>(rosNode.advertise<geometry_msgs::PoseStamped>("/positionLaserScanner", 10));

    initialIntensityThreshold = (*sc)["LaserLocalization"]->get<double>("LaserLocalization.initialIntensityThreshold", NULL);
}

MotionCalibrationLaser::~MotionCalibrationLaser()
{
    spinner.stop();
}

void MotionCalibrationLaser::onScan(const sensor_msgs::LaserScanConstPtr &laserScan)
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

    const float intensityThreshold = intensities[initialIntensityThreshold * numOfIntensities];
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

    if (pointGroups.size() != 2)
    {
        return;
    }

// TODO ifdef for debug must be corrected
#ifdef laserMotionCalibrationDebug
    geometry_msgs::PoseArray filteredPoses;
    geometry_msgs::PoseArray resultPoses;
    vector<std::shared_ptr<geometry::CNPoint2D>> pillarCenters;
    for (auto pointGroup : pointGroups)
    {
        geometry_msgs::Pose centerPose;
        geometry::CNPoint2D pillarCenter = *(pointGroup->getPillarCenter());
        pillarCenters.push_back(make_shared<geometry::CNPoint2D>(pillarCenter));
        centerPose.position = pillarCenter;
        cout << pillarCenter.length() << "m  ";
        resultPoses.poses.push_back(centerPose);

        for (auto point : pointGroup->points)
        {
            geometry_msgs::Point rosPoint;
            std::shared_ptr<geometry::CNPoint2D> coordinates = point.getXY();
            rosPoint.x = coordinates->x;
            rosPoint.y = coordinates->y;
            geometry_msgs::Pose pose;
            pose.position = rosPoint;
            filteredPoses.poses.push_back(pose);
        }
    }
    auto alloPos = calculateCoordinatesInPillarSystem(pillarCenters);

    geometry_msgs::Pose pose;
    pose.position = *alloPos.get();
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.pose = pose;
    cout << endl;
cout << "allopos: " << alloPos->x << "/" << alloPos->y << " (" << alloPos->length() << "m from first pillar)" << endl;
    cout << clock() - begin_time << " µs" << endl;

#endif

    poseStamped.header.frame_id = "laser";
    resultPoses.header.frame_id = "laser";   // for debugging purposes
    filteredPoses.header.frame_id = "laser"; // for debugging purposes
    resultsPublisher->publish(resultPoses);
    filteredPublisher->publish(filteredPoses);
    positionPublisher->publish(poseStamped);
}

shared_ptr<geometry::CNPoint2D> MotionCalibrationLaser::calculateCoordinatesInPillarSystem(vector<std::shared_ptr<geometry::CNPoint2D>> &pillarCenters)
{
    shared_ptr<geometry::CNPoint2D> left = pillarCenters[1];
    shared_ptr<geometry::CNPoint2D> right = pillarCenters[0];

    shared_ptr<geometry::CNPoint2D> lefToRight = right - left;
    shared_ptr<geometry::CNPoint2D> horizontal = make_shared<geometry::CNPoint2D>();

    horizontal->x = 1;
    horizontal->y = 0;

    double alpha = acos(lefToRight->x / lefToRight->length());

    geometry::CNPosition pos(left->x, left->y, alpha);
    shared_ptr<geometry::CNPoint2D> posPtr = make_shared<geometry::CNPoint2D>(geometry::CNPoint2D(0, 0));

    shared_ptr<geometry::CNPoint2D> allo = make_shared<geometry::CNPoint2D>();
    allo->x = -cos(alpha) * left->x + sin(alpha) * left->y;
    allo->y = -sin(alpha) * left->x - cos(alpha) * left->y;

    return allo;
}

std::vector<std::shared_ptr<LaserPointGroup>> MotionCalibrationLaser::getThresholdGroups(const sensor_msgs::LaserScanConstPtr &laserScan, double threshold)
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
            // cout << "[" << numberOfGroups << "] " << groupEnd - groupBegin + 1 << " scans (" << groupBegin << "-" << groupEnd << ")" << endl;
        }

        if (aboveThreshold)
        {
            // cout << "[" << numberOfGroups << "] Scan " << i << " (intensity=" << intensity << ", distance=" << laserScan->ranges[i] << ")" << endl;
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
    laserMotionCalibration::MotionCalibrationLaser motionCalibrationLaser = laserMotionCalibration::MotionCalibrationLaser(argc, argv);

    while (ros::ok())
    {
        ros::Duration(0.5).sleep();
    }

    return 0;
}
