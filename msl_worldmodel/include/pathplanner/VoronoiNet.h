/*
 * VoronoiNet.h
 *
 *  Created on: Apr 26, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_VORONOINET_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_VORONOINET_H_

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
typedef DelaunayAdaptionTraits::Point_2 Point_2;
typedef DelaunayAdaptionTraits::Site_2 Site_2;

#include <vector>
#include <sstream>
#include <iostream>

#include "container/CNPoint2D.h"
#include "pathplanner/SearchNode.h"
#include "pathplanner/evaluator/PathEvaluator.h"

namespace supplementary {
	class SystemConfig;
}

using namespace std;

namespace msl
{

	/**
	 * Class containing a CGAL voronoi diagramm and its status
	 */
	class PathEvaluator;
	class MSLWorldModel;
	class VoronoiNet
	{
	public:
		VoronoiNet(MSLWorldModel* wm);
		virtual ~VoronoiNet();
		/**
		 * generates a VoronoiDiagram and inserts given points
		 */
		shared_ptr<VoronoiDiagram> generateVoronoiDiagram(vector<geometry::CNPoint2D> points);
		/**
		 * gets the SearchNode with lowest dist to goal
		 */
		shared_ptr<SearchNode> getMin(shared_ptr<vector<shared_ptr<SearchNode>>> open);
		/**
		 * gets the closest vertex to a given point
		 */
		shared_ptr<VoronoiDiagram::Vertex> findClosestVertexToOwnPos(shared_ptr<geometry::CNPoint2D> ownPos);
		/**
		 * expands a SearchNode
		 */
		void expandNode(shared_ptr<SearchNode> currentNode,shared_ptr<vector<shared_ptr<SearchNode>>> open,
							shared_ptr<vector<shared_ptr<SearchNode>>> closed, shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<PathEvaluator> eval);
		/**
		 * return the sites near an egde defined by 2 points
		 * @param v1 VoronoiDiagram::Vertex
		 * @param v2 VoronoiDiagram::Vertex
		 * @returnpair<shared_ptr<Point_2>, shared_ptr<Point_2>>
		 */
		pair<shared_ptr<geometry::CNPoint2D>, shared_ptr<geometry::CNPoint2D>> getSitesNextToHalfEdge(shared_ptr<geometry::CNPoint2D> v1, shared_ptr<geometry::CNPoint2D> v2);
		/**
		 * print the voronoi diagrams sites
		 */
		void printSites();
		/**
		 * print the voronoi diagrams vertices
		 */
		void printVertices();
		/**
		 * to string
		 */
		string toString();

		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getVerticesOfFace(shared_ptr<geometry::CNPoint2D> point);

		/**
		 * inserts sites into the VoronoiDiagram
		 */
		void insertPoints(vector<Site_2> points);

		void insertAdditionalPoints(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points);

		shared_ptr<VoronoiDiagram> getVoronoi();
		void setVoronoi(shared_ptr<VoronoiDiagram> voronoi);
		/**
		 * calculates distance between two points
		 */
		int calcDist(shared_ptr<geometry::CNPoint2D> ownPos, shared_ptr<geometry::CNPoint2D> vertexPoint);

		shared_ptr<VoronoiDiagram::Site_2> getSiteOfFace(VoronoiDiagram::Point_2 point);

		bool isOwnCellEdge(geometry::CNPoint2D pos, shared_ptr<SearchNode> currentNode, shared_ptr<SearchNode> nextNode);

		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > getTeamMatePositions();

		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > getObstaclePositions();

		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > getOpponentPositions();

		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > getSitePositions();

		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getTeamMateVertices(int teamMateId);

		void removeSites(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> sites);

	private:
		/**
		 * gets Vertices connected to SeachNode vertex
		 */
		vector<shared_ptr<SearchNode>> getNeighboredVertices(shared_ptr<SearchNode> currentNode);
		/**
		 * checks if a SearchNode is part of a vector
		 */
		bool contains(shared_ptr<vector<shared_ptr<SearchNode>>> vector, shared_ptr<SearchNode> vertex);


	protected:
		Kernel kernel;
		DelaunayTriangulation delaunayTriangulation;
		DelaunayAdaptionTraits delaunayTraits;
		DelaunayAdaptionPolicy delaunayPolicy;
		shared_ptr<VoronoiDiagram> voronoi;
		MSLWorldModel* wm;
		supplementary::SystemConfig* sc;
		mutex netMutex;
		/**
		 * true if own robot false otherwise
		 */
		map<shared_ptr<geometry::CNPoint2D>, int> pointRobotKindMapping;
	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_VORONOINET_H_ */
