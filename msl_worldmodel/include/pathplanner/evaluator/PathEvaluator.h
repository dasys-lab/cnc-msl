/*
 * PathEvaluator.h
 *
 *  Created on: May 14, 2015
 *      Author: Stefan Jakob
 */

# pragma once

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

#include <memory>
#include <ros/ros.h>
#include <vector>

#include "pathplanner/evaluator/IPathEvaluator.h"

namespace supplementary
{
class SystemConfig;
}
namespace msl
{
class VoronoiNet;
class SearchNode;
class PathEvaluator : public IPathEvaluator
{
  public:
    PathEvaluator();
    virtual ~PathEvaluator();

    virtual std::pair<double, double> eval(geometry::CNPointAllo goal, std::shared_ptr<SearchNode> currentNode,
                                           std::shared_ptr<SearchNode> nextNode, const VoronoiNet &voronoi) const;

    virtual std::pair<double, double> evalInitial(geometry::CNPointAllo startPos, geometry::CNPointAllo goal,
                                                  std::shared_ptr<SearchNode> nextNode, const VoronoiNet &voronoi,
                                                  std::shared_ptr<const std::vector<geometry::CNPointAllo>> lastPath,
                                                  nonstd::optional<geometry::CNPointAllo> lastTarget) const;

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
     * weight for inverted distance of obstacles to to voronoi edge
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
    supplementary::SystemConfig *sc;
};

} /* namespace msl */

