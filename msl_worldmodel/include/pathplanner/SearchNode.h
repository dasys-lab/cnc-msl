/*
 * SearchNode.h
 *
 *  Created on: Apr 24, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_SEARCHNODE_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_SEARCHNODE_H_

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

#include <limits.h>
#include <memory>
#include "container/CNPoint2D.h"

using namespace std;

namespace msl
{

	class SearchNode
	{
	public:
//		SearchNode();
		SearchNode(VoronoiDiagram::Halfedge_around_vertex_circulator edge, double cost, double heuristic, shared_ptr<SearchNode> predecessor);
		SearchNode(shared_ptr<Vertex> vertex, double cost, double heuristic, shared_ptr<SearchNode> predecessor);
		virtual ~SearchNode();
		/**
		 * gets the cost
		 * @return double
		 */
		double getCost();
		/**
		 * sets the cost
		 * @param cost double
		 */
		void setCost(double cost);

		/**
		 * gets the heuristic
		 * @return double
		 */
		double getHeuristic();
		/**
		 * sets the heuristic
		 * @param heuristic double
		 */
		void setHeuristic(double heuristic);
		/**
		 * gets the predecessor node
		 * @return shared_ptr<SearchNode>
		 */
		shared_ptr<SearchNode> getPredecessor();
		/**
		 * sets the predecessor node
		 * @param predecessor shared_ptr<SearchNode>
		 */
		void setPredecessor(shared_ptr<SearchNode> predecessor);
		/**
		 * gets the vertex
		 * @return shared_ptr<VoronoiDiagram::Vertex>
		 */
		VoronoiDiagram::Halfedge_around_vertex_circulator getEdge();
		/**
		 * sets the vertex
		 * @param vertex shared_ptr<VoronoiDiagram::Vertex>
		 */
		void setEdge(VoronoiDiagram::Halfedge_around_vertex_circulator edge);
		/**
		 * compares two SearchNodes, true if first has lower cost
		 * @param first shared_ptr<SearchNode>
		 * @param second shared_ptr<SearchNode>
		 * @return bool
		 */
		static bool compare(shared_ptr<SearchNode> first, shared_ptr<SearchNode> second);

		shared_ptr<geometry::CNPoint2D> getPoint();

		VoronoiDiagram::Halfedge_around_vertex_circulator getIncidentEdges();

		bool matches(shared_ptr<Vertex> vertex);

	private:
		shared_ptr<SearchNode> predecessor;

		double cost;
		double heuristic;
		VoronoiDiagram::Halfedge_around_vertex_circulator edge;

		bool initialEdge;
		shared_ptr<Vertex> vertex;
	};

} /* namespace alicaTests */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_SEARCHNODE_H_ */
