/*
 * Obstacles.h
 *
 *  Created on: Feb 11, 2016
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_OBSTACLEHANDLER_OBHANDLER_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_OBSTACLEHANDLER_OBHANDLER_H_

#include "SystemConfig.h"
#include "memory"
#include "vector"
#include "MSLFootballField.h"
#include "msl_sensor_msgs/CorrectedOdometryInfo.h"
#include "obstaclehandler/AnnotatedObstacleCluster.h"
#include "msl_msgs/Point2dInfo.h"
#include "msl_msgs/PositionInfo.h"
#include "MSLEnums.h"

namespace msl
{
	class MSLWorldModel;
	class Obstacles
	{
	public:
		Obstacles(MSLWorldModel* wm, int ringbufferLength);
		virtual ~Obstacles();
		/// <summary> Merges all obstacles into lists which are requiered by
		/// different Modules (path planner, standard situation behaviours, delaunay generator etc.) </summary>
		/// <param name="myObstacles">
		/// A <see cref="List"/> of ego centric obstacles. (usual from the worldmodel).
		/// </param>
		void handleObstacles(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> myObstacles);

		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> clusterPoint2D (shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> obstacles, double varianceThreshold);

	private:
		void clusterAnnotatedObstacles();
		void setupAnnotatedObstacles(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> ownObs,
				                                            shared_ptr<msl_sensor_msgs::CorrectedOdometryInfo> myOdo);
		void processNegSupporter(shared_ptr<geometry::CNPosition> myPosition);
		bool leftOf(double angle1, double angle2);

		supplementary::SystemConfig* sc;
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
		MSLFootballField* field;
		MSLWorldModel* wm;
		AnnotatedObstacleClusterPool* pool;
		shared_ptr<vector<AnnotatedObstacleCluster*>> clusterArray;
		shared_ptr<vector<AnnotatedObstacleCluster*>> newClusterArray;
		double distance(msl_msgs::Point2dInfo point, msl_msgs::PositionInfo pos);
	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_OBSTACLEHANDLER_OBHANDLER_H_ */
