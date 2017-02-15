#pragma once

#include "InformationElement.h"
#include "MSLEnums.h"
#include "MSLFootballField.h"
#include <SystemConfig.h>
#include <cnc_geometry/CNRobot.h>
#include <InfoBuffer.h>

#include <msl_msgs/Point2dInfo.h>
#include <msl_msgs/PositionInfo.h>
#include <msl_sensor_msgs/CorrectedOdometryInfo.h>
#include <msl_sensor_msgs/ObstacleInfo.h>
#include <msl_sensor_msgs/WorldModelData.h>
#include <obstaclehandler/AnnotatedObstacleCluster.h>
#include <memory>
#include <vector>

namespace msl
{
class MSLWorldModel;
class Obstacles
{
  public:
    Obstacles(MSLWorldModel *wm, int ringbufferLength);
    virtual ~Obstacles();
    /// <summary> Merges all obstacles into lists which are requiered by
    /// different Modules (path planner, standard situation behaviours, delaunay generator etc.) </summary>
    /// <param name="myObstacles">
    /// A <see cref="List"/> of ego centric obstacles. (usual from the worldmodel).
    /// </param>
    void handleObstacles(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> myObstacles);
    void processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data);
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> clusterPoint2D(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> obstacles, double varianceThreshold);
    shared_ptr<vector<shared_ptr<geometry::CNRobot>>> getAlloObstacles(int index = 0);
    shared_ptr<vector<shared_ptr<geometry::CNRobot>>> getAlloObstaclesWithMe(int index = 0);
    shared_ptr<vector<shared_ptr<geometry::CNRobot>>> getEgoObstacles(int index = 0);
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getAlloObstaclePoints(int index = 0);
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getEgoObstaclePoints(int index = 0);
    // TODO change to raw
    shared_ptr<vector<msl_sensor_msgs::ObstacleInfo>> getEgoVisionObstacles(int index = 0);
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getEgoVisionObstaclePoints(int index = 0);
    double getObstacleRadius();

    shared_ptr<geometry::CNPoint2D> getBiggestFreeGoalAreaMidPoint();
    double getDistanceToObstacle(shared_ptr<geometry::CNPoint2D> target);

  private:
    void clusterAnnotatedObstacles();
    void setupAnnotatedObstacles(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ownObs, shared_ptr<msl_sensor_msgs::CorrectedOdometryInfo> myOdo);
    void processNegSupporter(shared_ptr<geometry::CNPosition> myPosition);
    bool leftOf(double angle1, double angle2);

    InfoBuffer<InformationElement<vector<msl_sensor_msgs::ObstacleInfo>>> obstacles;

    supplementary::SystemConfig *sc;
    double DENSITY;
    double VARIANCE_THRESHOLD;
    double TERRITORY_RADIUS;
    double SIGHT_RADIUS; // how far an obstacle can be seen proper
    double FIELD_TOL;
    double POS_CERTAINTY_TH_CLUSTERING;
    double POS_CERTAINTY_HYS;
    double DFLT_OB_RADIUS;
    double DFLT_ROB_RADIUS;
    double OBSTACLE_MAP_OUT_TOLERANCE;
    double LOCALIZATION_SUCCESS_CONFIDENCE;
    MSLWorldModel *wm;
    AnnotatedObstacleClusterPool *pool;
    shared_ptr<vector<AnnotatedObstacleCluster *>> clusterArray;
    shared_ptr<vector<AnnotatedObstacleCluster *>> newClusterArray;
    InfoBuffer<InformationElement<vector<shared_ptr<geometry::CNRobot>>>> obstaclesEgoClustered;
    InfoBuffer<InformationElement<vector<shared_ptr<geometry::CNRobot>>>> obstaclesAlloClustered;
    InfoBuffer<InformationElement<vector<shared_ptr<geometry::CNRobot>>>> obstaclesAlloClusteredWithMe;
    unsigned long maxInformationAge = 1000000000;
};

} /* namespace msl */
