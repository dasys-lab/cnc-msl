/*
 * PathEvaluator.h
 *
 *  Created on: May 14, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_PATHEVALUATOR_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_PATHEVALUATOR_H_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DelaunayTriangulation> DelaunayAdaptionTraits;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DelaunayTriangulation> DelaunayAdaptionPolicy;
typedef CGAL::Voronoi_diagram_2<DelaunayTriangulation, DelaunayAdaptionTraits, DelaunayAdaptionPolicy> VoronoiDiagram;
typedef VoronoiDiagram::Vertex Vertex;

#include <vector>
#include <memory>
#include "container/CNPoint2D.h"
#include "pathplanner/VoronoiNet.h"
#include <SystemConfig.h>
#include <ros/ros.h>
#include "msl_msgs/VoronoiNetInfo.h"
#include "pathplanner/evaluator/IPathEvaluator.h"
namespace msl
{
	class VoronoiNet;
	class PathEvaluator : IPathEvaluator
	{
	public:
		PathEvaluator();
		virtual ~PathEvaluator();
		virtual double eval(shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal,
							shared_ptr<SearchNode> currentNode, shared_ptr<SearchNode> nextNode,
							VoronoiNet* voronoi = nullptr,
							shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path = nullptr, shared_ptr<geometry::CNPoint2D> lastTarget = nullptr);

	protected:
		/**
		 * Diameter of a robot
		 */
		double robotDiameter;
		/**
		 * additional corridor with for the corridor check to ensure that no obstacle is near the path
		 */
		double additionalCorridorWidth;
		/**
		 * weigt for inverted distance of obstalces to to voronoi edge
		 */
		double obstacleDistanceWeight;
		/**
		 * weight for the length of the path
		 */
		double pathLengthWeight;
		/**
		 * weight for angle between 2 edges
		 */
		double pathAngleWeight;
		/**
		 * weight for the deviation of path start
		 */
		double pathDeviationWeight;
		ros::Publisher voronoiPub;
		ros::NodeHandle n;
		supplementary::SystemConfig* sc;

		double distanceTo(shared_ptr<Vertex> v1, shared_ptr<Vertex> v2);

	};

}/* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_PATHEVALUATOR_H_ */
