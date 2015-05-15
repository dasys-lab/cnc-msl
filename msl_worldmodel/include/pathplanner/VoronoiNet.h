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

#include <SystemConfig.h>
#include "container/CNPoint2D.h"
#include "pathplanner/SearchNode.h"
#include "pathplanner/VoronoiStatus.h"

using namespace std;

namespace msl
{

	/**
	 * Class containing a CGAL voronoi diagramm and its status
	 */
	class MSLWorldModel;
	class VoronoiNet
	{
	public:
		VoronoiNet(MSLWorldModel* wm);
		virtual ~VoronoiNet();
		/**
		 * generates a VoronoiDiagram and inserts given points
		 */
		shared_ptr<VoronoiDiagram> generateVoronoiDiagram(vector<CNPoint2D> points);
		/**
		 * gets the SearchNode with lowest dist to goal
		 */
		shared_ptr<SearchNode> getMin(shared_ptr<vector<shared_ptr<SearchNode>>> open);
		/**
		 * gets the closest vertex to a given point
		 */
		shared_ptr<VoronoiDiagram::Vertex> findClosestVertexToOwnPos(Point_2 ownPos);
		/**
		 * expands a SearchNode
		 */
		void expandNode(shared_ptr<SearchNode> currentNode,shared_ptr<vector<shared_ptr<SearchNode>>> open,
							shared_ptr<vector<shared_ptr<SearchNode>>> closed, Point_2 goal);
		/**
		 * gets the status of the VoronoiDiagram
		 */
		VoronoiStatus getStatus();
		/**
		 * sets the status of the VoronoiDiagram
		 */
		void setStatus(VoronoiStatus status);
		/**
		 * return the sites near an egde defined by 2 points
		 * @param v1 VoronoiDiagram::Vertex
		 * @param v2 VoronoiDiagram::Vertex
		 * @returnpair<shared_ptr<Point_2>, shared_ptr<Point_2>>
		 */
		pair<shared_ptr<Point_2>, shared_ptr<Point_2>> getSitesNextToHalfEdge(shared_ptr<VoronoiDiagram::Vertex> v1, shared_ptr<VoronoiDiagram::Vertex> v2);
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

		shared_ptr<vector<shared_ptr<VoronoiDiagram::Vertex>>> getVerticesOfFace(VoronoiDiagram::Point_2 point);

		/**
		 * inserts sites into the VoronoiDiagram
		 */
		void insertPoints(vector<Site_2> points);

		shared_ptr<VoronoiDiagram> getVoronoi();
		void setVoronoi(shared_ptr<VoronoiDiagram> voronoi);
		/**
		 * calculates distance between two points
		 */
		int calcDist(Point_2 ownPos, Point_2 vertexPoint);

		shared_ptr<VoronoiDiagram::Site_2> getSiteOfFace(VoronoiDiagram::Point_2 point);

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
		VoronoiStatus status;
		supplementary::SystemConfig* sc;
		mutex netMutex;
	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_VORONOINET_H_ */
