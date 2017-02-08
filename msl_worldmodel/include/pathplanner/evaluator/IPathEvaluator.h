/*
 * IPathEvaluator.h
 *
 *  Created on: Aug 17, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_IPATHEVALUATOR_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_IPATHEVALUATOR_H_
#include <memory>

using namespace std;
namespace geometry
{
class CNPoint2D;
}
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
    virtual pair<double, double> eval(shared_ptr<geometry::CNPoint2D> goal, shared_ptr<SearchNode> currentNode, shared_ptr<SearchNode> nextNode,
                                      VoronoiNet *voronoi) = 0;

    virtual pair<double, double> evalInitial(shared_ptr<geometry::CNPoint2D> startPos, shared_ptr<geometry::CNPoint2D> goal, shared_ptr<SearchNode> nextNode,
                                             VoronoiNet *voronoi, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> lastPath,
                                             shared_ptr<geometry::CNPoint2D> lastTarget) = 0;
};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_IPATHEVALUATOR_H_ */
