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
//		delete this->wm;
	}

};

/**
 * Tests the plan parser with some nice plans
 */
TEST_F(PathPlannerTest, pathPlanner)
{
	msl::MSLFootballField* field = msl::MSLFootballField::getInstance();
	shared_ptr<msl::VoronoiNet> net = this->wm->pathPlanner.getCurrentVoronoiNet();
	shared_ptr<geometry::CNPoint2D> startPos = field->posOwnPenaltyMarker();
	shared_ptr<geometry::CNPoint2D> goalPos = field->posOppPenaltyMarker();
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path = nullptr;
	shared_ptr<msl::PathEvaluator> eval = make_shared<msl::PathEvaluator>(msl::PathEvaluator(&(wm->pathPlanner)));
	EXPECT_TRUE(net == nullptr);
	shared_ptr<geometry::CNPoint2D> startPoint = make_shared<geometry::CNPoint2D>(0, 0);
	shared_ptr<geometry::CNPoint2D> goalPoint = make_shared<geometry::CNPoint2D>(1000, 1000);
	net = this->wm->pathPlanner.getArtificialObjectNet();
	EXPECT_TRUE(net != nullptr);
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
	for(int i = 0; i < 10; i++)
	{
		bool alreadyIn = false;
		auto point = make_shared<geometry::CNPoint2D>(rand() % (int)field->FieldLength - (int)field->FieldLength / 2, rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
		for (auto it = points->begin(); it != points->end(); it++)
		{
			//TODO needs to be checked
			if (abs((*it)->x - point->x) < 250 && abs((*it)->y - point->y) < 250)
			{
				alreadyIn = true;
				break;
			}
		}
		if (!alreadyIn)
		{
			points->push_back(point);
		}
		else
		{
			i--;
		}
		alreadyIn = false;
	}
	cout << "####################### 10 Obstacles #######################" << endl;
	ros::Time t1 = ros::Time::now();
	cout << "before insert : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
	net->insertAdditionalPoints(points);
	ros::Time t2 = ros::Time::now();
	cout << "after insert : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
	cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
	EXPECT_EQ(10, net->getObstaclePositions()->size());
	t1 = ros::Time::now();
	cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
	path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
	t2 = ros::Time::now();
	cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
	cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
	cout << "Path length: " << path->size() << endl;
	net->clearVoronoiNet();
	points->clear();

	for(int i = 0; i < 25; i++)
	{
		bool alreadyIn = false;
		auto point = make_shared<geometry::CNPoint2D>(rand() % (int)field->FieldLength - (int)field->FieldLength / 2, rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
		for (auto it = points->begin(); it != points->end(); it++)
		{
			//TODO needs to be checked
			if (abs((*it)->x - point->x) < 250 && abs((*it)->y - point->y) < 250)
			{
				alreadyIn = true;
				break;
			}
		}
		if (!alreadyIn)
		{
			points->push_back(point);
		}
		else
		{
			i--;
		}
		alreadyIn = false;
	}
	cout << "####################### 25 Obstacles #######################" << endl;
	t1 = ros::Time::now();
	cout << "before insert : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
	net->insertAdditionalPoints(points);
	t2 = ros::Time::now();
	cout << "after insert : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
	cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
	EXPECT_EQ(25, net->getObstaclePositions()->size());
	t1 = ros::Time::now();
	cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
	path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
	t2 = ros::Time::now();
	cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
	cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
	cout << "Path length: " << path->size() << endl;
	net->clearVoronoiNet();
	points->clear();

	for(int i = 0; i < 50; i++)
	{
		bool alreadyIn = false;
		auto point = make_shared<geometry::CNPoint2D>(rand() % (int)field->FieldLength - (int)field->FieldLength / 2, rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
		for (auto it = points->begin(); it != points->end(); it++)
		{
			//TODO needs to be checked
			if (abs((*it)->x - point->x) < 250 && abs((*it)->y - point->y) < 250)
			{
				alreadyIn = true;
				break;
			}
		}
		if (!alreadyIn)
		{
			points->push_back(point);
		}
		else
		{
			i--;
		}
		alreadyIn = false;
	}
	t1 = ros::Time::now();
	cout << "####################### 50 Obstacles #######################" << endl;
	cout << "before insert : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
	net->insertAdditionalPoints(points);
	t2 = ros::Time::now();
	cout << "after insert : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
	cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
	EXPECT_EQ(50, net->getObstaclePositions()->size());
	t1 = ros::Time::now();
	cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
	path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
	t2 = ros::Time::now();
	cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
	cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
	cout << "Path length: " << path->size() << endl;
	net->clearVoronoiNet();
	points->clear();

	for(int i = 0; i < 100; i++)
	{

		bool alreadyIn = false;
		auto point = make_shared<geometry::CNPoint2D>(rand() % (int)field->FieldLength - (int)field->FieldLength / 2, rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
		for (auto it = points->begin(); it != points->end(); it++)
		{
			//TODO needs to be checked
			if (abs((*it)->x - point->x) < 250 && abs((*it)->y - point->y) < 250)
			{
				alreadyIn = true;
				break;
			}
		}
		if (!alreadyIn)
		{
			points->push_back(point);
		}
		else
		{
			i--;
		}
		alreadyIn = false;
	}
	cout << "####################### 100 Obstacles #######################" << endl;
	t1 = ros::Time::now();
	cout << "before insert : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
	net->insertAdditionalPoints(points);
	t2 = ros::Time::now();
	cout << "after insert : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
	cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
	EXPECT_EQ(100, net->getObstaclePositions()->size());
	t1 = ros::Time::now();
	cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
	path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
	t2 = ros::Time::now();
	cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
	cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
	cout << "Path length: " << path->size() << endl;
	net->clearVoronoiNet();
	points->clear();

	for(int i = 0; i < 250; i++)
	{
		bool alreadyIn = false;
		auto point = make_shared<geometry::CNPoint2D>(rand() % (int)field->FieldLength - (int)field->FieldLength / 2, rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
		for (auto it = points->begin(); it != points->end(); it++)
		{
			//TODO needs to be checked
			if (abs((*it)->x - point->x) < 250 && abs((*it)->y - point->y) < 250)
			{
				alreadyIn = true;
				break;
			}
		}
		if (!alreadyIn)
		{
			points->push_back(point);
		}
		else
		{
			i--;
		}
		alreadyIn = false;
	}
	cout << "####################### 250 Obstacles #######################" << endl;
	t1 = ros::Time::now();
	cout << "before insert : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
	net->insertAdditionalPoints(points);
	t2 = ros::Time::now();
	cout << "after insert : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
	cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
	EXPECT_EQ(250, net->getObstaclePositions()->size());
	t1 = ros::Time::now();
	cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
	path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
	t2 = ros::Time::now();
	cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
	cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
	cout << "Path length: " << path->size() << endl;
	net->clearVoronoiNet();
	points->clear();

	for(int i = 0; i < 500; i++)
	{
		bool alreadyIn = false;
		auto point = make_shared<geometry::CNPoint2D>(rand() % (int)field->FieldLength - (int)field->FieldLength / 2, rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
		for (auto it = points->begin(); it != points->end(); it++)
		{
			//TODO needs to be checked
			if (abs((*it)->x - point->x) < 250 && abs((*it)->y - point->y) < 250)
			{
				alreadyIn = true;
				break;
			}
		}
		if (!alreadyIn)
		{
			points->push_back(point);
		}
		else
		{
			i--;
		}
		alreadyIn = false;
	}
	cout << "####################### 500 Obstacles #######################" << endl;
	t1 = ros::Time::now();
	cout << "before insert : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
	net->insertAdditionalPoints(points);
	t2 = ros::Time::now();
	cout << "after insert : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
	cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
	EXPECT_EQ(500, net->getObstaclePositions()->size());
	t1 = ros::Time::now();
	cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
	path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
	t2 = ros::Time::now();
	cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
	cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
	cout << "Path length: " << path->size() << endl;
	net->clearVoronoiNet();
	points->clear();

	for(int i = 0; i < 1000; i++)
	{
		bool alreadyIn = false;
		auto point = make_shared<geometry::CNPoint2D>(rand() % (int)field->FieldLength - (int)field->FieldLength / 2, rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
		for (auto it = points->begin(); it != points->end(); it++)
		{
			//TODO needs to be checked
			if (abs((*it)->x - point->x) < 250 && abs((*it)->y - point->y) < 250)
			{
				alreadyIn = true;
				break;
			}
		}
		if (!alreadyIn)
		{
			points->push_back(point);
		}
		else
		{
			i--;
		}
		alreadyIn = false;
	}
	cout << "####################### 1000 Obstacles #######################" << endl;
	t1 = ros::Time::now();
	cout << "before insert : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
	net->insertAdditionalPoints(points);
	t2 = ros::Time::now();
	cout << "after insert : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
	cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
	EXPECT_EQ(1000, net->getObstaclePositions()->size());
	t1 = ros::Time::now();
	cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
	path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
	t2 = ros::Time::now();
	cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
	cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
	cout << "Path length: " << path->size() << endl;
	net->clearVoronoiNet();
	points->clear();

}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "PathPlanner");
	bool ret = RUN_ALL_TESTS();
	ros::shutdown();
	return ret;
}

