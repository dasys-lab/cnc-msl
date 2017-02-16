#pragma once

#include "InformationElement.h"
#include "MSLEnums.h"
#include "MSLFootballField.h"
#include <InfoBuffer.h>
#include <SystemConfig.h>
#include <cnc_geometry/CNRobot.h>

#include <memory>
#include <msl_msgs/Point2dInfo.h>
#include <msl_msgs/PositionInfo.h>
#include <msl_sensor_msgs/CorrectedOdometryInfo.h>
#include <msl_sensor_msgs/ObstacleInfo.h>
#include <msl_sensor_msgs/WorldModelData.h>
#include <obstaclehandler/AnnotatedObstacleCluster.h>
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
    void handleObstacles(std::shared_ptr<std::vector<geometry::CNPointEgo>> myObstacles);
    void processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data);
    std::shared_ptr<std::vector<geometry::CNPoint2D>>
    clusterPoint2D(std::shared_ptr<std::vector<std::shared_ptr<geometry::CNPoint2D>>> obstacles,
                   double varianceThreshold);
    std::shared_ptr<std::vector<geometry::CNRobot>> getAlloObstacles(int index = 0);
    std::shared_ptr<std::vector<geometry::CNRobot>> getAlloObstaclesWithMe(int index = 0);
    std::shared_ptr<std::vector<geometry::CNRobot>> getEgoObstacles(int index = 0);
    std::shared_ptr<std::vector<geometry::CNPointAllo>> getAlloObstaclePoints(int index = 0);
    std::shared_ptr<std::vector<geometry::CNPointEgo>> getEgoObstaclePoints(int index = 0);
    // TODO change to raw
    std::shared_ptr<std::vector<msl_sensor_msgs::ObstacleInfo>> getEgoVisionObstacles(int index = 0);
    std::shared_ptr<std::vector<geometry::CNPointEgo>> getEgoVisionObstaclePoints(int index = 0);
    double getObstacleRadius();

    geometry::CNPoint2D getBiggestFreeGoalAreaMidPoint();
    double getDistanceToObstacle(geometry::CNPoint2D target);

  private:
    void clusterAnnotatedObstacles();
    void setupAnnotatedObstacles(std::shared_ptr<std::vector<geometry::CNPoint2D>> ownObs,
                                 msl_sensor_msgs::CorrectedOdometryInfo myOdo);
    void processNegSupporter(geometry::CNPosition myPosition);
    bool leftOf(double angle1, double angle2);

    InfoBuffer<InformationElement<std::vector<msl_sensor_msgs::ObstacleInfo>>> obstacles;

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
    std::shared_ptr<std::vector<AnnotatedObstacleCluster *>> clusterArray;
    std::shared_ptr<std::vector<AnnotatedObstacleCluster *>> newClusterArray;
    InfoBuffer<std::shared_ptr<const std::vector<geometry::CNRobot>>> obstaclesEgoClustered;
    InfoBuffer<std::shared_ptr<const std::vector<geometry::CNRobot>>> obstaclesAlloClustered;
    InfoBuffer<std::shared_ptr<const std::vector<geometry::CNRobot>>> obstaclesAlloClusteredWithMe;
    unsigned long maxInformationAge = 1000000000;
};

} /* namespace msl */
