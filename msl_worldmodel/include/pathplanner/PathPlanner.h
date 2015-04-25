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
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>
#include <math.h>

#include "SystemConfig.h"
#include "pathplanner/SearchNode.h"

//namespaces
using namespace std;

namespace msl
{

	class MSLWorldModel;
	class PathPlanner
	{
	public:
		PathPlanner(MSLWorldModel* wm);
		virtual ~PathPlanner();
		VoronoiDiagram* generateVoronoiDiagram();
		void insertPoints(vector<Site_2> points);
		shared_ptr<vector<shared_ptr<Point_2>>> aStarSearch(Point_2 ownPos, Point_2 goal);

	private:
		shared_ptr<VoronoiDiagram::Vertex> findClosestVertexToOwnPos(Point_2 ownPos);
		int calcDist(Point_2 ownPos, Point_2 vertexPoint);
		bool checkGoal(shared_ptr<VoronoiDiagram::Vertex> vertex, Point_2 goal);
		shared_ptr<vector<shared_ptr<VoronoiDiagram::Vertex>>> getVerticesNearPoint(Point_2 point);
		shared_ptr<SearchNode> getMin(shared_ptr<vector<shared_ptr<SearchNode>>> open);
		void expandNode(shared_ptr<SearchNode> currentNode,shared_ptr<vector<shared_ptr<SearchNode>>> open,
						shared_ptr<vector<shared_ptr<SearchNode>>> closed, Point_2 goal);
		vector<shared_ptr<SearchNode>> getNeighboredVertices(shared_ptr<SearchNode> currentNode);
		bool contains(shared_ptr<vector<shared_ptr<SearchNode>>> vector, shared_ptr<SearchNode> vertex);

	protected:
		Kernel kernel;
		DelaunayTriangulation delaunayTriangulation;
		DelaunayAdaptionTraits delaunayTraits;
		DelaunayAdaptionPolicy delaunayPolicy;
		VoronoiDiagram voronoi;
		MSLWorldModel* wm;
		supplementary::SystemConfig* sc;
	};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_H_ */
