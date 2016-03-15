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
typedef VoronoiDiagram::Vertex Vertex;

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
#include <msl_sensor_msgs/WorldModelData.h>
#include "msl_msgs/CorridorCheck.h"
#include "msl_msgs/Point2dInfo.h"
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include "MSLFootballField.h"
#include "pathplanner/evaluator/PathEvaluator.h"
#include "MSLEnums.h"

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
		 * method for path planning
		 * @param voronoi shared_ptr<VoronoiNet>
		 * @param ownPos Point_2
		 * @param goal Point_2
		 * @return shared_ptr<vector<shared_ptr<Point_2>>>
		 */
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> plan(shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<IPathEvaluator> eval);

	/**
	 * processes the WorldModel msg
	 * @param msg msl_sensor_msgs::WorldModelDataPtr
	 */
	void prepareVoronoiDiagram();
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

	/**
	 * gets robot diameter
	 */
	double getRobotDiameter();
	/**
	 * gets pathDeviationWeight
	 */
	double getPathDeviationWeight();
	/**
	 * get dribbleAngleTolerance
	 */
	double getDribbleAngleTolerance();
	/**
	 * gets dribbleRotationWeight
	 */
	double getDribbleRotationWeight();
	/**
	 * gets additionalCorridorWidth
	 */
	double getAdditionalCorridorWidth();
	/**
	 * inserts a searchnode into a sorted vector
	 */
	static void insert(shared_ptr<vector<shared_ptr<SearchNode>>> vect, shared_ptr<SearchNode> currentNode);
	/**
	 * gets the base voronoi net with artificial obstacles
	 */
	shared_ptr<VoronoiNet> getArtificialObjectNet();
	/**
	 * gets artificial obstacles as goemtry::CNPonit2D
	 */
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getArtificialFieldSurroundingObs();
	/**
	 * gets last returned path
	 */
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getLastPath();
	/**
	 * gets last planning target
	 */
	shared_ptr<geometry::CNPoint2D> getLastTarget();

	bool contains(shared_ptr<vector<shared_ptr<SearchNode> > > vector, VoronoiDiagram::Halfedge_around_vertex_circulator edge);

	bool isAdmissableEdge(VoronoiDiagram::Halfedge_around_vertex_circulator incidentHalfEdge, shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<VoronoiNet> voronoi);
	/**
	 * expands a SearchNode
	 * @param currentNode shared_ptr<SearchNode>
	 * @param open shared_ptr<vector<shared_ptr<SearchNode>>>
	 * @param closed shared_ptr<vector<shared_ptr<SearchNode>>>
	 * @param startPos shared_ptr<geometry::CNPoint2D>
	 * @param goal shared_ptr<geometry::CNPoint2D>
	 * @param eval shared_ptr<PathEvaluator>
	 */
	void expandNode(shared_ptr<SearchNode> currentNode, shared_ptr<vector<shared_ptr<SearchNode>>> open,
			shared_ptr<vector<shared_ptr<SearchNode>>> closed, shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<IPathEvaluator> eval, shared_ptr<VoronoiNet> voronoi);

	/**
	 * checks if there is an obstacle inside the corridor
	 * @param voronoi shared_ptr<VoronoiNet>
	 * @param currentPos shared_ptr<geometry::CNPoint2D>
	 * @param goal shared_ptr<geometry::CNPoint2D>
	 * @param obstaclePoint shared_ptr<geometry::CNPoint2D>
	 * @return bool true if inside corridor false otherwise
	 */
	bool corridorCheck(shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> currentPos,
			shared_ptr<geometry::CNPoint2D> goal, shared_ptr<geometry::CNPoint2D> obstaclePoint);
	/**
	 * checks if there is an obstacle inside the corridor
	 * @param voronoi VoronoiNet*
	 * @param currentPos shared_ptr<geometry::CNPoint2D>
	 * @param goal shared_ptr<geometry::CNPoint2D>
	 * @param obstaclePoint shared_ptr<geometry::CNPoint2D>
	 * @return bool true if inside corridor false otherwise
	 */
	bool corridorCheck(VoronoiNet* voronoi, shared_ptr<geometry::CNPoint2D> currentPos,
			shared_ptr<geometry::CNPoint2D> goal, shared_ptr<geometry::CNPoint2D> obstaclePoint);

	/**
	 * checks if there is an obstacle inside the corridor
	 * @param voronoi shared_ptr<VoronoiNet>VoronoiNet*
	 * @param currentPos shared_ptr<geometry::CNPoint2D>
	 * @param goal shared_ptr<geometry::CNPoint2D>
	 * @param obstaclePoint shared_ptr<geometry::CNPoint2D>
	 * @return bool true if inside corridor false otherwise
	 */
	bool corridorCheckBall(shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> currentPos,
			shared_ptr<geometry::CNPoint2D> goal, shared_ptr<geometry::CNPoint2D> obstaclePoint);

private:
	/**
	 * initializes artificial objects and insert them into artificial object net
	 */
	void initializeArtificialObstacles();
	/**
	 * compares two search nodes
	 */
	static bool compare(shared_ptr<SearchNode> first, shared_ptr<SearchNode> second);
	/**
	 * checks if vertices of goal face are reached
	 */
	bool checkGoalVerticesReached(shared_ptr<vector<shared_ptr<Vertex> > > closestVerticesToGoal, shared_ptr<SearchNode> currentNode);
	/**
	 * helping method to debug the corridor check
	 */
	void sendCorridorCheck(vector<shared_ptr<geometry::CNPoint2D>> points);

	double distanceTo(shared_ptr<geometry::CNPoint2D> v1, shared_ptr<Vertex> v2);

	/**
	 * aStar search on a VoronoiDiagram
	 * @param voronoi shared_ptr<VoronoiNet>
	 * @param startPos Point_2
	 * @param goal Point_2
	 * @return shared_ptr<vector<shared_ptr<Point_2>>>
	 */
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> aStarSearch(shared_ptr<VoronoiNet> voronoi, shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<IPathEvaluator> pathEvaluator);

protected:
	MSLWorldModel* wm;
	MSLFootballField* field;
	int currentVoronoiPos;
	supplementary::SystemConfig* sc;
	mutex voronoiMutex;
	vector<shared_ptr<VoronoiNet>> voronoiDiagrams;
	double robotDiameter;
	shared_ptr<VoronoiNet> artificialObjectNet;
	double pathDeviationWeight;
	double dribble_rotationWeight;
	double dribble_angleTolerance;
	double minEdgeWidth;
	double corridorWidthDivisor;
	bool pathPlannerDebug;
	double marginToBlockedArea;
	double additionalCorridorWidth;
	shared_ptr<SearchNode> lastClosestNode;
	double snapDistance;
	double additionalBallCorridorWidth;
	double corridorWidthDivisorBall;

	ros::NodeHandle n;
	ros::Publisher corridorPub;
	shared_ptr<geometry::CNPoint2D> lastClosestPointToBlock;
	shared_ptr<geometry::CNPoint2D> lastTarget;
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> lastPath;
	/**
	 * check if the goal vertices are reached and if there is a corridor leading to the goal
	 */
	bool checkGoalReachable(shared_ptr<VoronoiNet> voronoi, shared_ptr<SearchNode> currentNode, shared_ptr<vector<shared_ptr<Vertex>>> closestVerticesToGoal, shared_ptr<geometry::CNPoint2D> goal);
};

}
/* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_H_ */
