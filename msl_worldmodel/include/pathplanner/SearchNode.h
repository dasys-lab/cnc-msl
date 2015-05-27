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

#include <limits.h>
#include <memory>
#include "container/CNPoint2D.h"

using namespace std;

namespace msl
{

	class SearchNode
	{
	public:
		SearchNode();
		SearchNode(shared_ptr<CNPoint2D> vertex, double cost, shared_ptr<SearchNode> predecessor);
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
		shared_ptr<CNPoint2D> getVertex();
		/**
		 * sets the vertex
		 * @param vertex shared_ptr<VoronoiDiagram::Vertex>
		 */
		void setVertex(shared_ptr<CNPoint2D> vertex);
		/**
		 * compares two SearchNodes, true if first has lower cost
		 * @param first shared_ptr<SearchNode>
		 * @param second shared_ptr<SearchNode>
		 * @return bool
		 */
		static bool compare(shared_ptr<SearchNode> first, shared_ptr<SearchNode> second);

	private:
		shared_ptr<SearchNode> predecessor;
		double cost;
		shared_ptr<CNPoint2D> vertex;
	};

} /* namespace alicaTests */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_SEARCHNODE_H_ */
