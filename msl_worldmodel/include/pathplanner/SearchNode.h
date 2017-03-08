#pragma once

#include <cnc_geometry/CNPointAllo.h>
#include <nonstd/optional.hpp>

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

#include <limits.h>
#include <memory>

namespace msl
{
/**
 * Node used inside the A*-Algorithm
 */
class SearchNode
{
  public:
    //		SearchNode();
    SearchNode(VoronoiDiagram::Halfedge_around_vertex_circulator edge, double cost, double heuristic, std::shared_ptr<SearchNode> predecessor);
    SearchNode(Vertex vertex, double cost, double heuristic, std::shared_ptr<SearchNode> predecessor);
    virtual ~SearchNode();
    /**
     * Gets the cost
     * @return double
     */
    double getCost() const;
    /**
     * Sets the cost
     * @param double cost
     */
    void setCost(double cost);

    /**
     * Gets the heuristic vlaue
     * @return double
     */
    double getHeuristic() const;
    /**
     * Sets the heuristic value
     * @param double heuristic
     */
    void setHeuristic(double heuristic);
    /**
     * Gets the predecessor node
     * @return shared_ptr<SearchNode>
     */
    std::shared_ptr<const SearchNode> getPredecessor() const;

    /**
     * Gets the predecessor node
     * @return shared_ptr<SearchNode>
     */
    std::shared_ptr<SearchNode> getPredecessor();
    /**
     * Sets the predecessor node
     * @param shared_ptr<SearchNode> predecessor
     */
    void setPredecessor(std::shared_ptr<SearchNode> predecessor);
    /**
     * Gets the Voronoi Edge
     * @return VoronoiDiagram::Halfedge_around_vertex_circulator
     */
    VoronoiDiagram::Halfedge_around_vertex_circulator getEdge() const;
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
    static bool compare(std::shared_ptr<const SearchNode> first, std::shared_ptr<const SearchNode> second);

    /**
     * Get starting point of the edge
     * @return the starting point of the edge
     */
    geometry::CNPointAllo getPoint();

    /**
     * Gets incident edges to the voronoi vertex
     * @return VoronoiDiagram::Halfedge_around_vertex_circulator
     */
    VoronoiDiagram::Halfedge_around_vertex_circulator getIncidentEdges();

    bool matches(const Vertex &vertex);

  private:
    std::shared_ptr<SearchNode> predecessor;

    double cost;
    double heuristic;
    VoronoiDiagram::Halfedge_around_vertex_circulator edge;

    bool initialEdge;
    nonstd::optional<Vertex> vertex;
};

} /* namespace msl */
