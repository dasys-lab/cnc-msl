/*
 * IPathEvaluator.h
 *
 *  Created on: Aug 17, 2015
 *      Author: Stefan Jakob
 */

#pragma once

#include <cnc_geometry/CNPointAllo.h>
#include <nonstd/optional.hpp>

#include <memory>

namespace msl
{
class SearchNode;
class VoronoiNet;
/**
 * Interface class for Pathplanning evaluators
 */
class IPathEvaluator
{
  public:
    virtual ~IPathEvaluator()
    {
    }
    virtual std::pair<double, double> eval(geometry::CNPointAllo goal, std::shared_ptr<SearchNode> currentNode,
                                           std::shared_ptr<SearchNode> nextNode, const VoronoiNet &voronoi) const = 0;

    virtual std::pair<double, double> evalInitial(geometry::CNPointAllo startPos, geometry::CNPointAllo goal,
                                                  std::shared_ptr<SearchNode> nextNode, const VoronoiNet &voronoi,
                                                  std::shared_ptr<const std::vector<geometry::CNPointAllo>> lastPath,
                                                  nonstd::optional<geometry::CNPointAllo> lastTarget) const = 0;
};

} /* namespace msl */
