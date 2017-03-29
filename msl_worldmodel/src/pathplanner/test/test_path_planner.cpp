/*
 * test_path_planner.cpp
 *
 *  Created on: May 22, 2015
 *      Author: Stefan Jakob
 */

#include "SystemConfig.h"
#include <gtest/gtest.h>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <typeinfo>

#include "MSLWorldModel.h"
#include "container/CNPoint2D.h"
#include "pathplanner/PathPlanner.h"
#include "pathplanner/evaluator/PathEvaluator.h"
using namespace std;

class PathPlannerTest : public ::testing::Test
{

  protected:
    supplementary::SystemConfig *sc;
    msl::MSLWorldModel *wm;
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
    msl::MSLFootballField *field = msl::MSLFootballField::getInstance();
    shared_ptr<msl::VoronoiNet> net = this->wm->pathPlanner.getCurrentVoronoiNet();
    shared_ptr<geometry::CNPoint2D> startPos = field->posOwnPenaltyMarker();
    shared_ptr<geometry::CNPoint2D> goalPos = field->posOppPenaltyMarker();
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path = nullptr;
    shared_ptr<msl::PathEvaluator> eval = make_shared<msl::PathEvaluator>(msl::PathEvaluator());
    EXPECT_TRUE(net == nullptr);
    shared_ptr<geometry::CNPoint2D> startPoint = make_shared<geometry::CNPoint2D>(0, 0);
    shared_ptr<geometry::CNPoint2D> goalPoint = make_shared<geometry::CNPoint2D>(1000, 1000);
    net = this->wm->pathPlanner.getArtificialObjectNet();
    auto artificialObs = this->wm->pathPlanner.getArtificialFieldSurroundingObs();
    int artObsSize = artificialObs->size();
    EXPECT_TRUE(net != nullptr);
    cout << "####################### 10 Obstacles #######################" << endl;
    double avgTime = 0;
    for (int j = 0; j < 120; j++)
    {
        //		cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& iteration " << j + 1 << " &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
        shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        net->clearVoronoiNet();
        net->insertAdditionalPoints(artificialObs);
        for (int i = 0; i < 10; i++)
        {
            bool alreadyIn = false;
            auto point = make_shared<geometry::CNPoint2D>(rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
                                                          rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
            for (auto it = points->begin(); it != points->end(); it++)
            {
                // TODO needs to be checked
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

        ros::Time t1 = ros::Time::now();
        net->insertAdditionalPoints(points);

        ros::Time t2 = ros::Time::now();
        unsigned long timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff insert 10: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
        //				<< "ns" << endl;
        EXPECT_EQ(10 + artObsSize, net->getObstaclePositions()->size());

        t1 = ros::Time::now();
        path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
        t2 = ros::Time::now();
        timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        avgTime += timeDiff;
        //		cout << /*"Time diff plan 10: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << */timeDiff
        //				<< "ns" << endl;

        //		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> toBeDeleted = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        //		for (int it = 0; it < 2; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 10 20%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 4; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 10 40%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 6; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 10 60%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 8; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 10 80%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 10; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 10 100%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        net->clearVoronoiNet();
        //		toBeDeleted->clear();
        points->clear();
    }
    cout << "%%%%%%%%% avg Time 10 " << avgTime / 120 << " " << avgTime << endl;
    avgTime = 0;
    //	cout << "####################### 15 Obstacles #######################" << endl;
    for (int j = 0; j < 120; j++)
    {
        //		cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& iteration " << j + 1 << " &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
        //		cout << endl;
        net->clearVoronoiNet();
        net->insertAdditionalPoints(artificialObs);
        shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        for (int i = 0; i < 15; i++)
        {
            bool alreadyIn = false;
            auto point = make_shared<geometry::CNPoint2D>(rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
                                                          rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
            for (auto it = points->begin(); it != points->end(); it++)
            {
                // TODO needs to be checked
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

        ros::Time t1 = ros::Time::now();
        net->insertAdditionalPoints(points);

        ros::Time t2 = ros::Time::now();
        unsigned long timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff insert 15: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
        //				<< "ns" << endl;
        EXPECT_EQ(15 + artObsSize, net->getObstaclePositions()->size());

        t1 = ros::Time::now();
        path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
        t2 = ros::Time::now();
        timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        avgTime += timeDiff;
        //		cout << "Time diff plan 15: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
        //				<< "ns" << endl;
        //
        //		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> toBeDeleted = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        //		for (int it = 0; it < 3; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 15 20%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 6; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 15 40%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 9; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 15 60%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 12; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 15 80%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 15; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 15 100%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        net->clearVoronoiNet();
        //		toBeDeleted->clear();
        points->clear();
        //
    }
    cout << "%%%%%%%%% avg Time 15 " << avgTime / 120 << " " << avgTime << endl;
    avgTime = 0;
    //	cout << "####################### 20 Obstacles #######################" << endl;
    for (int j = 0; j < 120; j++)
    {
        //		cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& iteration " << j + 1 << " &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
        //		cout << endl;
        net->clearVoronoiNet();
        net->insertAdditionalPoints(artificialObs);
        shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        for (int i = 0; i < 20; i++)
        {
            bool alreadyIn = false;
            auto point = make_shared<geometry::CNPoint2D>(rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
                                                          rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
            for (auto it = points->begin(); it != points->end(); it++)
            {
                // TODO needs to be checked
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

        ros::Time t1 = ros::Time::now();
        net->insertAdditionalPoints(points);

        ros::Time t2 = ros::Time::now();
        unsigned long timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff insert 20: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
        //				<< "ns" << endl;
        EXPECT_EQ(20 + artObsSize, net->getObstaclePositions()->size());

        t1 = ros::Time::now();
        path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
        t2 = ros::Time::now();
        timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        avgTime += timeDiff;
        //		cout << "Time diff plan 20: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
        //				<< "ns" << endl;
        //
        //		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> toBeDeleted = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        //		for (int it = 0; it < 4; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 20 20%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 8; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 20 40%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 12; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 20 60%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 16; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 20 80%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 20; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 20 100%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        net->clearVoronoiNet();
        //		toBeDeleted->clear();
        points->clear();
        //
    }
    cout << "%%%%%%%%% avg Time 20 " << avgTime / 120 << " " << avgTime << endl;
    avgTime = 0;
    //	cout << "####################### 25 Obstacles #######################" << endl;
    for (int j = 0; j < 120; j++)
    {
        //		cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& iteration " << j + 1 << " &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
        //		cout << endl;
        net->clearVoronoiNet();
        net->insertAdditionalPoints(artificialObs);
        shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        for (int i = 0; i < 25; i++)
        {
            bool alreadyIn = false;
            auto point = make_shared<geometry::CNPoint2D>(rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
                                                          rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
            for (auto it = points->begin(); it != points->end(); it++)
            {
                // TODO needs to be checked
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

        ros::Time t1 = ros::Time::now();
        net->insertAdditionalPoints(points);

        ros::Time t2 = ros::Time::now();
        unsigned long timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff insert 25: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
        //				<< "ns" << endl;
        EXPECT_EQ(25 + artObsSize, net->getObstaclePositions()->size());

        t1 = ros::Time::now();
        path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
        t2 = ros::Time::now();
        timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        avgTime += timeDiff;
        //		cout << "Time diff plan 25: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
        //				<< "ns" << endl;
        //
        //		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> toBeDeleted = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        //		for (int it = 0; it < 5; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 25 20%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 10; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 25 40%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 15; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 25 60%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 20; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 25 80%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 25; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 25 100%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        net->clearVoronoiNet();
        //		toBeDeleted->clear();
        points->clear();
        //
    }
    cout << "%%%%%%%%% avg Time 25 " << avgTime / 120 << " " << avgTime << endl;
    avgTime = 0;

    //	cout << "####################### 50 Obstacles #######################" << endl;
    for (int j = 0; j < 120; j++)
    {
        //		cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& iteration " << j + 1 << " &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
        //		cout << endl;
        net->clearVoronoiNet();
        net->insertAdditionalPoints(artificialObs);
        shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        for (int i = 0; i < 50; i++)
        {
            bool alreadyIn = false;
            auto point = make_shared<geometry::CNPoint2D>(rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
                                                          rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
            for (auto it = points->begin(); it != points->end(); it++)
            {
                // TODO needs to be checked
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

        ros::Time t1 = ros::Time::now();
        net->insertAdditionalPoints(points);

        ros::Time t2 = ros::Time::now();
        unsigned long timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff insert 50: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
        //				<< "ns" << endl;
        EXPECT_EQ(50 + artObsSize, net->getObstaclePositions()->size());

        t1 = ros::Time::now();
        path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
        t2 = ros::Time::now();
        timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        avgTime += timeDiff;
        //		cout << "Time diff plan 50: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
        //				<< "ns" << endl;

        //		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> toBeDeleted = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        //		for (int it = 0; it < 10; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 50 20%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 20; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 50 40%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 30; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 50 60%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 40; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 50 80%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 50; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 50 100%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        net->clearVoronoiNet();
        //		toBeDeleted->clear();
        points->clear();
        //
    }
    cout << "%%%%%%%%% avg Time 50 " << avgTime / 120 << " " << avgTime << endl;
    avgTime = 0;
    //	cout << "####################### 100 Obstacles #######################" << endl;
    for (int j = 0; j < 120; j++)
    {
        //		cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& iteration " << j + 1 << " &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
        //		cout << endl;
        net->clearVoronoiNet();
        net->insertAdditionalPoints(artificialObs);
        shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        for (int i = 0; i < 100; i++)
        {
            bool alreadyIn = false;
            auto point = make_shared<geometry::CNPoint2D>(rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
                                                          rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
            for (auto it = points->begin(); it != points->end(); it++)
            {
                // TODO needs to be checked
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

        ros::Time t1 = ros::Time::now();
        net->insertAdditionalPoints(points);

        ros::Time t2 = ros::Time::now();
        unsigned long timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff insert 100: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
        //				<< "ns" << endl;
        EXPECT_EQ(100 + artObsSize, net->getObstaclePositions()->size());

        t1 = ros::Time::now();
        path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
        t2 = ros::Time::now();
        timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        avgTime += timeDiff;
        //		cout << "Time diff plan 100: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
        //				<< "ns" << endl;
        //
        //		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> toBeDeleted = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        //		for (int it = 0; it < 20; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 100 20%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 40; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 100 40%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 60; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 100 60%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 80; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 100 80%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		net->insertAdditionalPoints(artificialObs);
        //		toBeDeleted->clear();
        //		net->insertAdditionalPoints(points);
        //		for (int it = 0; it < 100; it++)
        //		{
        //			toBeDeleted->push_back(points->at(it));
        //		}
        //		t1 = ros::Time::now();
        //		net->removeSites(toBeDeleted);
        //		t2 = ros::Time::now();
        //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
        //		cout << "Time diff delete 100 100%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
        //				<< timeDiff << "ns" << endl;
        //		net->clearVoronoiNet();
        //		toBeDeleted->clear();
        //		points->clear();
        //
    }
    cout << "%%%%%%%%% avg Time 100 " << avgTime / 120 << " " << avgTime << endl;
    avgTime = 0;
    //	cout << "####################### 250 Obstacles #######################" << endl;
    //	for (int j = 0; j < 120; j++)
    //	{
    //		cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& iteration " << j + 1 << " &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
    //		cout << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    //		for (int i = 0; i < 250; i++)
    //		{
    //			bool alreadyIn = false;
    //			auto point = make_shared<geometry::CNPoint2D>(
    //					rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
    //					rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
    //			for (auto it = points->begin(); it != points->end(); it++)
    //			{
    //				//TODO needs to be checked
    //				if (abs((*it)->x - point->x) < 250 && abs((*it)->y - point->y) < 250)
    //				{
    //					alreadyIn = true;
    //					break;
    //				}
    //			}
    //			if (!alreadyIn)
    //			{
    //				points->push_back(point);
    //			}
    //			else
    //			{
    //				i--;
    //			}
    //			alreadyIn = false;
    //		}
    //
    //		ros::Time t1 = ros::Time::now();
    //		net->insertAdditionalPoints(points);
    //
    //		ros::Time t2 = ros::Time::now();
    //		unsigned long timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff insert 250: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
    //				<< "ns" << endl;
    //		EXPECT_EQ(250 + artObsSize, net->getObstaclePositions()->size());
    //
    //		t1 = ros::Time::now();
    //		path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff plan 250: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
    //				<< "ns" << endl;
    //
    //		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> toBeDeleted = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    //		for (int it = 0; it < 50; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 250 20%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		toBeDeleted->clear();
    //		net->insertAdditionalPoints(points);
    //		for (int it = 0; it < 100; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 250 40%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		toBeDeleted->clear();
    //		net->insertAdditionalPoints(points);
    //		for (int it = 0; it < 150; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 250 60%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		toBeDeleted->clear();
    //		net->insertAdditionalPoints(points);
    //		for (int it = 0; it < 200; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 250 80%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		toBeDeleted->clear();
    //		net->insertAdditionalPoints(points);
    //		for (int it = 0; it < 250; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 250 100%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		toBeDeleted->clear();
    //		points->clear();
    //
    //	}
    cout << "%%%%%%%%% avg Time 250 " << avgTime / 120 << " " << avgTime << endl;
    avgTime = 0;
    //	cout << "####################### 500 Obstacles #######################" << endl;
    //	for (int j = 0; j < 120; j++)
    //	{
    //		cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& iteration " << j + 1 << " &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
    //		cout << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    //		for (int i = 0; i < 500; i++)
    //		{
    //			bool alreadyIn = false;
    //			auto point = make_shared<geometry::CNPoint2D>(
    //					rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
    //					rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
    //			for (auto it = points->begin(); it != points->end(); it++)
    //			{
    //				//TODO needs to be checked
    //				if (abs((*it)->x - point->x) < 250 && abs((*it)->y - point->y) < 250)
    //				{
    //					alreadyIn = true;
    //					break;
    //				}
    //			}
    //			if (!alreadyIn)
    //			{
    //				points->push_back(point);
    //			}
    //			else
    //			{
    //				i--;
    //			}
    //			alreadyIn = false;
    //		}
    //
    //		ros::Time t1 = ros::Time::now();
    //		net->insertAdditionalPoints(points);
    //
    //		ros::Time t2 = ros::Time::now();
    //		unsigned long timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff insert 500: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
    //				<< "ns" << endl;
    //		EXPECT_EQ(500 + artObsSize, net->getObstaclePositions()->size());
    //
    //		t1 = ros::Time::now();
    //		path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff plan 500: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
    //				<< "ns" << endl;
    //
    //		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> toBeDeleted = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    //		for (int it = 0; it < 100; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 500 20%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		toBeDeleted->clear();
    //		net->insertAdditionalPoints(points);
    //		for (int it = 0; it < 200; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 500 40%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		toBeDeleted->clear();
    //		net->insertAdditionalPoints(points);
    //		for (int it = 0; it < 300; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 500 60%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		toBeDeleted->clear();
    //		net->insertAdditionalPoints(points);
    //		for (int it = 0; it < 400; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 500 80%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		toBeDeleted->clear();
    //		net->insertAdditionalPoints(points);
    //		for (int it = 0; it < 500; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 500 100%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		toBeDeleted->clear();
    //		points->clear();
    //
    //	}
    cout << "%%%%%%%%% avg Time 500 " << avgTime / 120 << " " << avgTime << endl;
    avgTime = 0;
    //	cout << "####################### 1000 Obstacles #######################" << endl;
    //	for (int j = 0; j < 120; j++)
    //	{
    //		cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& iteration " << j + 1 << " &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
    //		cout << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    //		for (int i = 0; i < 1000; i++)
    //		{
    //			bool alreadyIn = false;
    //			auto point = make_shared<geometry::CNPoint2D>(
    //					rand() % (int)field->FieldLength - (int)field->FieldLength / 2,
    //					rand() % (int)field->FieldWidth - (int)field->FieldWidth / 2);
    //			for (auto it = points->begin(); it != points->end(); it++)
    //			{
    //				//TODO needs to be checked
    //				if (abs((*it)->x - point->x) < 250 && abs((*it)->y - point->y) < 250)
    //				{
    //					alreadyIn = true;
    //					break;
    //				}
    //			}
    //			if (!alreadyIn)
    //			{
    //				points->push_back(point);
    //			}
    //			else
    //			{
    //				i--;
    //			}
    //			alreadyIn = false;
    //		}
    //
    //		ros::Time t1 = ros::Time::now();
    //		net->insertAdditionalPoints(points);
    //
    //		ros::Time t2 = ros::Time::now();
    //		unsigned long timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff insert 1000: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		EXPECT_EQ(1000 + artObsSize, net->getObstaclePositions()->size());
    //
    //		t1 = ros::Time::now();
    //		path = wm->pathPlanner.plan(net, startPos, goalPos, eval);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff plan 1000: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms " << timeDiff
    //				<< "ns" << endl;
    //
    //		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> toBeDeleted = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    //		for (int it = 0; it < 200; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 1000 20%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		toBeDeleted->clear();
    //		net->insertAdditionalPoints(points);
    //		for (int it = 0; it < 400; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 1000 40%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		toBeDeleted->clear();
    //		net->insertAdditionalPoints(points);;
    //		for (int it = 0; it < 600; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 1000 60%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		toBeDeleted->clear();
    //		net->insertAdditionalPoints(points);
    //		for (int it = 0; it < 800; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 1000 80%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		net->insertAdditionalPoints(artificialObs);
    //		toBeDeleted->clear();
    //		net->insertAdditionalPoints(points);
    //		for (int it = 0; it < 1000; it++)
    //		{
    //			toBeDeleted->push_back(points->at(it));
    //		}
    //		t1 = ros::Time::now();
    //		net->removeSites(toBeDeleted);
    //		t2 = ros::Time::now();
    //		timeDiff = (t2.sec * 1000000000UL + t2.nsec) - (t1.sec * 1000000000UL + t1.nsec);
    //		cout << "Time diff delete 1000 100%: " << timeDiff / 1000000000UL << "s " << timeDiff / 1000000UL << "ms "
    //				<< timeDiff << "ns" << endl;
    //		net->clearVoronoiNet();
    //		toBeDeleted->clear();
    //		points->clear();
    //
    //	}
    cout << "%%%%%%%%% avg Time 1000 " << avgTime / 120 << " " << avgTime << endl;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "PathPlanner");
    bool ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
