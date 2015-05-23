/*
 * test_path_planner.cpp
 *
 *  Created on: May 22, 2015
 *      Author: Stefan Jakob
 */


#include "SystemConfig.h"
#include <gtest/gtest.h>
#include <map>
#include <list>
#include <typeinfo>
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>

#include "MSLWorldModel.h"
#include "pathplanner/PathPlanner.h"
#include "container/CNPoint2D.h"
#include "pathplanner/evaluator/PathEvaluator.h"
using namespace std;

class PathPlannerTest : public ::testing::Test
{

protected:
	supplementary::SystemConfig* sc;
	msl::MSLWorldModel* wm;

	virtual void SetUp()
	{
		// determine the path to the test config
		string path = supplementary::FileSystem::getSelfPath();
		int place = path.rfind("devel");
		path = path.substr(0, place);
		path = path + "src/cnc-msl/msl_worldmodel/src/pathplanner/test";

		// bring up the SystemConfig with the corresponding path
		sc = supplementary::SystemConfig::getInstance();
		sc->setRootPath(path);
		sc->setConfigPath(path + "/etc");
		sc->setHostname("nase");
		this->wm = new msl::MSLWorldModel();

	}

	virtual void TearDown()
	{
		sc->shutdown();
		delete this->wm;
	}

};

/**
 * Tests the plan parser with some nice plans
 */
TEST_F(PathPlannerTest, pathPlanner)
{
	shared_ptr<msl::VoronoiNet> net = this->wm->pathPlanner.getCurrentVoronoiNet();
	EXPECT_TRUE(net == nullptr);
	msl::CNPoint2D startPoint(0, 0);
	msl::CNPoint2D goalPoint(1000, 2000);
	msl_sensor_msgs::WorldModelDataPtr msg = boost::make_shared<msl_sensor_msgs::WorldModelData>();
	this->wm->pathPlanner.processWorldModelData(msg);
	net = this->wm->pathPlanner.getCurrentVoronoiNet();
	EXPECT_TRUE(net != nullptr);
	shared_ptr<vector<shared_ptr<msl::CNPoint2D>>> path = this->wm->pathPlanner.aStarSearch(net,startPoint, goalPoint, new msl::PathEvaluator(&(this->wm->pathPlanner)));
	EXPECT_EQ(path->size(), 1);
	msl_sensor_msgs::ObstacleInfo info;
	info.x = -100;
	info.y = -100;
	msg->obstacles.push_back(info);
	this->wm->pathPlanner.processWorldModelData(msg);
	net = this->wm->pathPlanner.getCurrentVoronoiNet();
	path = this->wm->pathPlanner.aStarSearch(net,startPoint, goalPoint, new msl::PathEvaluator(&(this->wm->pathPlanner)));
	EXPECT_EQ(path->size(), 1);
	msl_sensor_msgs::ObstacleInfo info2;
	info2.x = 500;
	info2.y = 1000;
	msl_sensor_msgs::ObstacleInfo info3;
	info3.x = 1500;
	info3.y = 1000;
	msl_sensor_msgs::ObstacleInfo info4;
	info4.x = 750;
	info4.y = 800;
	msl_sensor_msgs::ObstacleInfo info5;
	info5.x = 2000;
	info5.y = 1000;
	msl_sensor_msgs::ObstacleInfo info6;
	info6.x = 900;
	info6.y = 1000;
	msl_sensor_msgs::ObstacleInfo info7;
	info7.x = 300;
	info7.y = 200;
	msl_sensor_msgs::ObstacleInfo info8;
	info8.x = 600;
	info8.y = 100;
	msl_sensor_msgs::ObstacleInfo info9;
	info9.x = 300;
	info9.y = 300;
	msl_sensor_msgs::ObstacleInfo info10;
	info10.x = 250;
	info10.y = 375;
	msl_sensor_msgs::ObstacleInfo info11;
	info11.x = 1100;
	info11.y = 1250;
	msg->obstacles.push_back(info2);
	msg->obstacles.push_back(info3);
	msg->obstacles.push_back(info4);
	msg->obstacles.push_back(info5);
	msg->obstacles.push_back(info6);
	msg->obstacles.push_back(info7);
	msg->obstacles.push_back(info8);
	msg->obstacles.push_back(info9);
	msg->obstacles.push_back(info10);
	msg->obstacles.push_back(info11);
	this->wm->pathPlanner.processWorldModelData(msg);
	net = this->wm->pathPlanner.getCurrentVoronoiNet();
	path = this->wm->pathPlanner.aStarSearch(net,startPoint, goalPoint, new msl::PathEvaluator(&(this->wm->pathPlanner)));
	EXPECT_EQ(path->size(), 1);
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "PathPlanner");
	bool ret = RUN_ALL_TESTS();
	ros::shutdown();
	return ret;
}

