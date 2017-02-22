#pragma once

#include <pathplanner/evaluator/PathEvaluator.h>

namespace msl
{

class Evaluator : public PathEvaluator
{
  public:
    Evaluator();
    virtual ~Evaluator();
    double eval(shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal,
                shared_ptr<SearchNode> currentNode, shared_ptr<SearchNode> nextNode, VoronoiNet *voronoi = nullptr,
                shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path = nullptr,
                shared_ptr<geometry::CNPoint2D> lastTarget = nullptr);

  private:
    double clearSpaceWeight;
};

} /* namespace msl */
