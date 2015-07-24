/*
 * PathPlanner.h
 *
 *  Created on: Feb 24, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_H_

//includes for CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>


/**
 * typedefs to short CGAL's class names e.g.CGAL::Voronoi_diagram_2<CGAL::Delaunay_triangulation_2<CGAL::Exact_predicates_inexact_constructions_kernel>,
CGAL::Delaunay_triangulation_adaptation_traits_2<CGAL::Delaunay_triangulation_2<CGAL::Exact_predicates_inexact_constructions_kernel>>,
CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<CGAL::Delaunay_triangulation_2<CGAL::Exact_predicates_inexact_constructions_kernel>>>
to VoronoiDiagram
 */
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DelaunayTriangulation> DelaunayAdaptionTraits;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DelaunayTriangulation> DelaunayAdaptionPolicy;
typedef CGAL::Voronoi_diagram_2<DelaunayTriangulation, DelaunayAdaptionTraits, DelaunayAdaptionPolicy> VoronoiDiagram;
typedef DelaunayAdaptionTraits::Point_2 Point_2;
typedef DelaunayAdaptionTraits::Site_2 Site_2;

//other includes
#include <ros/ros.h>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>
#include <math.h>
#include <mutex>

#include "SystemConfig.h"
#include "pathplanner/SearchNode.h"
#include "pathplanner/VoronoiNet.h"
#include "pathplanner/VoronoiStatus.h"
#include <msl_sensor_msgs/WorldModelData.h>
#include "msl_msgs/CorridorCheck.h"
#include "msl_msgs/Point2dInfo.h"
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include "MSLFootballField.h"
#include "pathplanner/evaluator/PathEvaluator.h"


//namespaces
using namespace std;

namespace msl
{

	class MSLWorldModel;
	class VoronoiNet;
	class PathEvaluator;
	class PathPlanner
	{
	public:
		PathPlanner(MSLWorldModel* wm, int count);
		virtual ~PathPlanner();
		/**
		 * aStar search on a VoronoiDiagram
		 * @param voronoi shared_ptr<VoronoiNet>
		 * @param ownPos Point_2
		 * @param goal Point_2
		 * @return shared_ptr<vector<shared_ptr<Point_2>>>
		 */
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> plan(shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> ownPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<PathEvaluator> eval);

		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> aStarSearch(shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<PathEvaluator> eval);
		/**
		 * processes the WorldModel msg
		 * @param msg msl_sensor_msgs::WorldModelDataPtr
		 */
		void processWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg);
		/**
		 * gets all saved VoronoiNets
		 * @return vector<shared_ptr<VoronoiNet>>
		 */
		vector<shared_ptr<VoronoiNet>> getVoronoiNets();
		/**
		 * gets latest accesable VoronoiNet
		 * @return shared_ptr<VoronoiNet>
		 */
		shared_ptr<VoronoiNet> getCurrentVoronoiNet();

		double getRobotDiameter();
		double getPathDeviationWeight();
		double getDribbleAngleTolerance();
		double getDribbleRotationWeight();
		static void insert(shared_ptr<vector<shared_ptr<SearchNode>>> vect, shared_ptr<SearchNode> currentNode);
	    shared_ptr<VoronoiNet> getArtificialObjectNet();
	    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getArtificialObstacles();
	    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getLastPath();
	    MSLWorldModel* getWm();
		bool corridorCheck(shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> currentPos,
							shared_ptr<geometry::CNPoint2D> goal, shared_ptr<geometry::CNPoint2D> obstaclePoint);

	private:
		void initializeArtificialObstacles();
		static bool compare(shared_ptr<SearchNode> first, shared_ptr<SearchNode> second);
		bool checkGoalVerticesReached(const shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > >& closestVerticesToGoal, const shared_ptr<SearchNode>& currentNode, bool found);
		void sendCorridorCheck(vector<shared_ptr<geometry::CNPoint2D>> points);

	protected:
		MSLWorldModel* wm;
		int currentVoronoiPos;
		supplementary::SystemConfig* sc;
		mutex voronoiMutex;
		vector<shared_ptr<VoronoiNet>> voronoiDiagrams;
		double robotDiameter;
		shared_ptr<VoronoiNet> artificialObjectNet;
		double pathDeviationWeight;
		double dribble_rotationWeight;
		double dribble_angleTolerance;
		double corridorWidthDivisor;
		bool pathPlannerDebug;
		double additionalCorridorWidth;
		double snapDistance;
		ros::NodeHandle n;
		ros::Publisher corridorPub;
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> lastPath;
		bool checkGoalReachable(shared_ptr<VoronoiNet> voronoi, shared_ptr<SearchNode> currentNode, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> closestVerticesToGoal, shared_ptr<geometry::CNPoint2D> goal);
	};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_H_ */
