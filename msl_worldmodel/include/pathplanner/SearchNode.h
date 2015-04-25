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

using namespace std;

namespace msl
{

	class SearchNode
	{
	public:
		SearchNode();
		SearchNode(shared_ptr<VoronoiDiagram::Vertex> vertex, double cost, shared_ptr<SearchNode> predecessor);
		virtual ~SearchNode();
		double getCost();
		void setCost(double cost);
		const shared_ptr<SearchNode> getPredecessor();
		void setPredecessor(shared_ptr<SearchNode> predecessor);
		shared_ptr<VoronoiDiagram::Vertex> getVertex();
		void setVertex(shared_ptr<VoronoiDiagram::Vertex> vertex);
		static bool compare(shared_ptr<SearchNode> first, shared_ptr<SearchNode> second);

	private:
		shared_ptr<SearchNode> predecessor;
		double cost;
		shared_ptr<VoronoiDiagram::Vertex> vertex;
	};

} /* namespace alicaTests */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_SEARCHNODE_H_ */
