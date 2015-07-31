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
		this->wm = msl::MSLWorldModel::get();

	}

	virtual void TearDown()
	{
		sc->shutdown();
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
	auto artificialObs = this->wm->pathPlanner.getArtificialObstacles();
	int artObsSize = artificialObs->size();
	EXPECT_TRUE(net != nullptr);
	for (int k = 1; k <= 5; k++)
	{
		cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% deleting " << 20 * k << "% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
		cout << endl;
		for (int j = 0; j < 120; j++)
		{
			cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& iteration " << j + 1 << " &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
			cout << endl;
			net->clearVoronoiNet();
			net->insertAdditionalPoints(artificialObs);
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
			for (int i = 0; i < 10; i++)
			{
				bool alreadyIn = false;
				auto point = make_shared<geometry::CNPoint2D>(
						rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
						rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
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
			unsigned long timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			EXPECT_EQ(10 + artObsSize, net->getObstaclePositions()->size());

			t1 = ros::Time::now();
			cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
			t2 = ros::Time::now();
			cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			cout << "Path length: " << path->size() << endl;

			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> toBeDeleted = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
			for (int it = 0; it <  k * 2; it++)
			{
				toBeDeleted->push_back(points->at(it));
			}
			cout << "deleting " << toBeDeleted->size() << " points" << endl;
			t1 = ros::Time::now();
			cout << "before deleting : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			net->removeSites(toBeDeleted);
			t2 = ros::Time::now();
			cout << "after deleting : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			net->clearVoronoiNet();
			points->clear();
			toBeDeleted->clear();

			net->insertAdditionalPoints(artificialObs);
			points = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
			for (int i = 0; i < 15; i++)
			{
				bool alreadyIn = false;
				auto point = make_shared<geometry::CNPoint2D>(
						rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
						rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
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
			cout << "####################### 15 Obstacles #######################" << endl;
			t1 = ros::Time::now();
			cout << "before insert : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			net->insertAdditionalPoints(points);
			t2 = ros::Time::now();
			cout << "after insert : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			EXPECT_EQ(15 + artObsSize, net->getObstaclePositions()->size());

			t1 = ros::Time::now();
			cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
			t2 = ros::Time::now();
			cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			cout << "Path length: " << path->size() << endl;

			for (int it = 0; it < k * 3; it++)
			{
				toBeDeleted->push_back(points->at(it));
			}
			cout << "deleting " << toBeDeleted->size() << " points" << endl;
			t1 = ros::Time::now();
			cout << "before deleting : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			net->removeSites(toBeDeleted);
			t2 = ros::Time::now();
			cout << "after deleting : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			net->clearVoronoiNet();
			points->clear();
			toBeDeleted->clear();

			net->insertAdditionalPoints(artificialObs);
			points = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
			for (int i = 0; i < 20; i++)
			{
				bool alreadyIn = false;
				auto point = make_shared<geometry::CNPoint2D>(
						rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
						rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
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
			cout << "####################### 20 Obstacles #######################" << endl;
			t1 = ros::Time::now();
			cout << "before insert : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			net->insertAdditionalPoints(points);
			t2 = ros::Time::now();
			cout << "after insert : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			EXPECT_EQ(20 + artObsSize, net->getObstaclePositions()->size());

			t1 = ros::Time::now();
			cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
			t2 = ros::Time::now();
			cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			cout << "Path length: " << path->size() << endl;

			for (int it = 0; it < k * 4; it++)
			{
				toBeDeleted->push_back(points->at(it));
			}
			cout << "deleting " << toBeDeleted->size() << " points" << endl;
			t1 = ros::Time::now();
			cout << "before deleting : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			net->removeSites(toBeDeleted);
			t2 = ros::Time::now();
			cout << "after deleting : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			net->clearVoronoiNet();
			points->clear();
			toBeDeleted->clear();

			net->insertAdditionalPoints(artificialObs);
			for (int i = 0; i < 25; i++)
			{
				bool alreadyIn = false;
				auto point = make_shared<geometry::CNPoint2D>(
						rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
						rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
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
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			EXPECT_EQ(25 + artObsSize, net->getObstaclePositions()->size());

			t1 = ros::Time::now();
			cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
			t2 = ros::Time::now();
			cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			cout << "Path length: " << path->size() << endl;

			for (int it = 0; it < k * 5; it++)
			{
				toBeDeleted->push_back(points->at(it));
			}
			cout << "deleting " << toBeDeleted->size() << " points" << endl;
			t1 = ros::Time::now();
			cout << "before deleting : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			net->removeSites(toBeDeleted);
			t2 = ros::Time::now();
			cout << "after deleting : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			net->clearVoronoiNet();
			points->clear();
			toBeDeleted->clear();

			net->insertAdditionalPoints(artificialObs);
			for (int i = 0; i < 50; i++)
			{
				bool alreadyIn = false;
				auto point = make_shared<geometry::CNPoint2D>(
						rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
						rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
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
			cout << "####################### 50 Obstacles #######################" << endl;
			t1 = ros::Time::now();
			cout << "before insert : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			net->insertAdditionalPoints(points);
			t2 = ros::Time::now();
			cout << "after insert : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			EXPECT_EQ(50 + artObsSize, net->getObstaclePositions()->size());

			t1 = ros::Time::now();
			cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
			t2 = ros::Time::now();
			cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			cout << "Path length: " << path->size() << endl;

			for (int it = 0; it < k * 10; it++)
			{
				toBeDeleted->push_back(points->at(it));
			}
			cout << "deleting " << toBeDeleted->size() << " points" << endl;
			t1 = ros::Time::now();
			cout << "before deleting : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			net->removeSites(toBeDeleted);
			t2 = ros::Time::now();
			cout << "after deleting : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			net->clearVoronoiNet();
			points->clear();
			toBeDeleted->clear();

			net->insertAdditionalPoints(artificialObs);
			for (int i = 0; i < 100; i++)
			{

				bool alreadyIn = false;
				auto point = make_shared<geometry::CNPoint2D>(
						rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
						rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
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
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			EXPECT_EQ(100 + artObsSize, net->getObstaclePositions()->size());

			t1 = ros::Time::now();
			cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
			t2 = ros::Time::now();
			cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			cout << "Path length: " << path->size() << endl;

			for (int it = 0; it < k * 20; it++)
			{
				toBeDeleted->push_back(points->at(it));
			}
			cout << "deleting " << toBeDeleted->size() << " points" << endl;
			t1 = ros::Time::now();
			cout << "before deleting : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			net->removeSites(toBeDeleted);
			t2 = ros::Time::now();
			cout << "after deleting : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			net->clearVoronoiNet();
			points->clear();
			toBeDeleted->clear();

			net->insertAdditionalPoints(artificialObs);
			for (int i = 0; i < 250; i++)
			{
				bool alreadyIn = false;
				auto point = make_shared<geometry::CNPoint2D>(
						rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
						rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
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
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			EXPECT_EQ(250 + artObsSize, net->getObstaclePositions()->size());

			t1 = ros::Time::now();
			cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
			t2 = ros::Time::now();
			cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			cout << "Path length: " << path->size() << endl;

			for (int it = 0; it < k * 50; it++)
			{
				toBeDeleted->push_back(points->at(it));
			}
			cout << "deleting " << toBeDeleted->size() << " points" << endl;
			t1 = ros::Time::now();
			cout << "before deleting : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			net->removeSites(toBeDeleted);
			t2 = ros::Time::now();
			cout << "after deleting : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			net->clearVoronoiNet();
			points->clear();
			toBeDeleted->clear();

			net->insertAdditionalPoints(artificialObs);
			for (int i = 0; i < 500; i++)
			{
				bool alreadyIn = false;
				auto point = make_shared<geometry::CNPoint2D>(
						rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
						rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
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
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			EXPECT_EQ(500 + artObsSize, net->getObstaclePositions()->size());

			t1 = ros::Time::now();
			cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
			t2 = ros::Time::now();
			cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			cout << "Path length: " << path->size() << endl;

			for (int it = 0; it < k * 100; it++)
			{
				toBeDeleted->push_back(points->at(it));
			}
			cout << "deleting " << toBeDeleted->size() << " points" << endl;
			t1 = ros::Time::now();
			cout << "before deleting : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			net->removeSites(toBeDeleted);
			t2 = ros::Time::now();
			cout << "after deleting : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			net->clearVoronoiNet();
			points->clear();
			toBeDeleted->clear();

			net->insertAdditionalPoints(artificialObs);
			for (int i = 0; i < 1000; i++)
			{
				bool alreadyIn = false;
				auto point = make_shared<geometry::CNPoint2D>(
						rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
						rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
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
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			EXPECT_EQ(1000 + artObsSize, net->getObstaclePositions()->size());

			t1 = ros::Time::now();
			cout << "before planing : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
			t2 = ros::Time::now();
			cout << "after planing : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			cout << "Time diff: " << (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			cout << "Path length: " << path->size() << endl;

			for (int it = 0; it <  k * 200; it++)
			{
				toBeDeleted->push_back(points->at(it));
			}
			cout << "deleting " << toBeDeleted->size() << " points" << endl;
			t1 = ros::Time::now();
			cout << "before deleting : " << (t1.sec * 1000000000UL + t1.nsec) << endl;
			net->removeSites(toBeDeleted);
			t2 = ros::Time::now();
			cout << "after deleting : " << (t2.sec * 1000000000UL + t2.nsec) << endl;
			timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
			cout << "Time diff: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
					<< "ns" << endl;
			net->clearVoronoiNet();
			points->clear();
			toBeDeleted->clear();
		}
	}
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "PathPlanner");
	bool ret = RUN_ALL_TESTS();
	ros::shutdown();
	return ret;
}

