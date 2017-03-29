/*
 * SearchNode.h
 *
 *  Created on: Apr 24, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_SEARCHNODE_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_SEARCHNODE_H_

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Voronoi_diagram_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DelaunayTriangulation> DelaunayAdaptionTraits;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DelaunayTriangulation> DelaunayAdaptionPolicy;
typedef CGAL::Voronoi_diagram_2<DelaunayTriangulation, DelaunayAdaptionTraits, DelaunayAdaptionPolicy> VoronoiDiagram;
typedef VoronoiDiagram::Vertex Vertex;

#include "container/CNPoint2D.h"
#include <limits.h>
#include <memory>

using namespace std;

namespace msl
{
/**
 * Node used inside the A*-Algorithm
 */
class SearchNode
{
  public:
    //		SearchNode();
    SearchNode(VoronoiDiagram::Halfedge_around_vertex_circulator edge, double cost, double heuristic, shared_ptr<SearchNode> predecessor);
    SearchNode(shared_ptr<Vertex> vertex, double cost, double heuristic, shared_ptr<SearchNode> predecessor);
    virtual ~SearchNode();
    /**
     * Gets the cost
     * @return double
     */
    double getCost();
    /**
     * Sets the cost
     * @param double cost
     */
    void setCost(double cost);

    /**
     * Gets the heuristic vlaue
     * @return double
     */
    double getHeuristic();
    /**
     * Sets the heuristic value
     * @param double heuristic
     */
    void setHeuristic(double heuristic);
    /**
     * Gets the predecessor node
     * @return shared_ptr<SearchNode>
     */
    shared_ptr<SearchNode> getPredecessor();
    /**
     * Sets the predecessor node
     * @param shared_ptr<SearchNode> predecessor
     */
    void setPredecessor(shared_ptr<SearchNode> predecessor);
    /**
     * Gets the Voronoi Edge
     * @return VoronoiDiagram::Halfedge_around_vertex_circulator
     */
    VoronoiDiagram::Halfedge_around_vertex_circulator getEdge();
    /**
     * Sets the Voronoi Edge
     * @param VoronoiDiagram::Halfedge_around_vertex_circulator
     */
    void setEdge(VoronoiDiagram::Halfedge_around_vertex_circulator edge);
    /**
     * Compares two SearchNodes, true if first has lower cost
     * @param shared_ptr<SearchNode> first
     * @param shared_ptr<SearchNode> second
     * @return bool
     */
    static bool compare(shared_ptr<SearchNode> first, shared_ptr<SearchNode> second);

    /**
     * Get starting point of the edge
     * @return shared_ptr<geometry::CNPoint2D>
     */
    shared_ptr<geometry::CNPoint2D> getPoint();

    /**
     * Gets incident edges to the voronoi vertex
     * @return VoronoiDiagram::Halfedge_around_vertex_circulator
     */
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
