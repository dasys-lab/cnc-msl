#pragma once

#include <pathplanner/evaluator/PathEvaluator.h>

namespace msl
{

class Evaluator : public PathEvaluator
{
  public:
    Evaluator();
    virtual ~Evaluator();
    double eval(geometry::CNPointAllo startPos, geometry::CNPointAllo goal, std::shared_ptr<SearchNode> currentNode,
                std::shared_ptr<SearchNode> nextNode, std::shared_ptr<const VoronoiNet> voronoi = nullptr,
                std::shared_ptr<std::vector<geometry::CNPointAllo>> path = nullptr,
                nonstd::optional<geometry::CNPointAllo> lastTarget = nonstd::nullopt);

  private:
    double clearSpaceWeight;
};

} /* namespace msl */
