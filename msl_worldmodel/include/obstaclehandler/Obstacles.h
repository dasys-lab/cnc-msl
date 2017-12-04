#pragma once

#include "MSLEnums.h"
#include "MSLFootballField.h"
#include "obstaclehandler/AnnotatedObstacleCluster.h"
#include "obstaclehandler/CNRobotAllo.h"
#include "obstaclehandler/CNRobotEgo.h"

#include <SystemConfig.h>
#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPointEgo.h>
#include <nonstd/optional.hpp>
#include <supplementary/InfoBuffer.h>

#include <msl_msgs/Point2dInfo.h>
#include <msl_msgs/PositionInfo.h>
#include <msl_sensor_msgs/CorrectedOdometryInfo.h>
#include <msl_sensor_msgs/ObstacleInfo.h>
#include <msl_sensor_msgs/WorldModelData.h>

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

    /* ===== Data integration ===== */

    void processWorldModelData(msl_sensor_msgs::WorldModelData &data);

    /* ===== Buffer access ===== */

    // Raw Info
    const supplementary::InfoBuffer<std::shared_ptr<const std::vector<msl_sensor_msgs::ObstacleInfo>>> &getObstaclesInfoBuffer() const;

    // Raw Obstacles
    const supplementary::InfoBuffer<std::shared_ptr<const std::vector<geometry::CNPointAllo>>> &getRawObstaclesAlloBuffer() const;
    const supplementary::InfoBuffer<std::shared_ptr<const std::vector<geometry::CNPointEgo>>> &getRawObstaclesEgoBuffer() const;

    // Clustered
    const supplementary::InfoBuffer<std::shared_ptr<const std::vector<CNRobotAllo>>> &
    getClusteredObstaclesAlloBuffer() const;
    const supplementary::InfoBuffer<std::shared_ptr<const std::vector<CNRobotAllo>>> &
    getClusteredObstaclesAlloWithMeBuffer() const;
    const supplementary::InfoBuffer<std::shared_ptr<const std::vector<CNRobotEgo>>> &getClusteredObstaclesEgoBuffer() const;

    /* ===== Other functions ===== */

    double getObstacleRadius() const;
    nonstd::optional<geometry::CNPointEgo> getBiggestFreeGoalAreaMidPoint() const;
    double getDistanceToObstacle(geometry::CNPointEgo target) const;

  private:
    // Worldmodel
    MSLWorldModel *wm;

    // Config
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

    void handleObstacles(std::shared_ptr<const std::vector<geometry::CNPointEgo>> myObstacles);

    shared_ptr<vector<AnnotatedObstacleCluster *>>
    clusterAnnotatedObstacles(shared_ptr<vector<AnnotatedObstacleCluster *>> clusterArray);
    shared_ptr<vector<AnnotatedObstacleCluster *>>
    setupAnnotatedObstacles(std::shared_ptr<const std::vector<geometry::CNPointEgo>> ownObs,
                            msl_sensor_msgs::CorrectedOdometryInfo myOdo);
    // void processNegSupporter(geometry::CNPositionAllo myPosition); // TODO: remove?
    bool leftOf(double angle1, double angle2) const;

    AnnotatedObstacleClusterPool *pool;

    /* ===== Buffers ===== */

    // Raw Info
    supplementary::InfoBuffer<std::shared_ptr<const std::vector<msl_sensor_msgs::ObstacleInfo>>> obstaclesInfoBuffer;

    // Raw Obstacles
    supplementary::InfoBuffer<std::shared_ptr<const std::vector<geometry::CNPointAllo>>> rawObstaclesAlloBuffer;
    supplementary::InfoBuffer<std::shared_ptr<const std::vector<geometry::CNPointEgo>>> rawObstaclesEgoBuffer;

    // Clustered Obstacles
    supplementary::InfoBuffer<std::shared_ptr<const std::vector<CNRobotAllo>>> clusteredObstaclesAlloBuffer;
    supplementary::InfoBuffer<std::shared_ptr<const std::vector<CNRobotAllo>>> clusteredObstaclesAlloWithMeBuffer;
    supplementary::InfoBuffer<std::shared_ptr<const std::vector<CNRobotEgo>>> clusteredObstaclesEgoBuffer;

    /* ===== Validity Durations ===== */

    // TODO: add constant for each InfoBuffer (and add to Config?)
    supplementary::InfoTime maxInfoValidity = 1000000000;
};

} /* namespace msl */
