#pragma once

// includes for CGAL
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
typedef DelaunayAdaptionTraits::Point_2 Point_2;
typedef DelaunayAdaptionTraits::Site_2 Site_2;
typedef Kernel::Iso_rectangle_2 Iso_rectangle_2;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::Ray_2 Ray_2;
typedef Kernel::Line_2 Line_2;

#include "VoronoiNet.h"
#include "container/CNPoint2D.h"
#include "pathplanner/evaluator/PathEvaluator.h"
#include <memory>
#include <pathplanner/PathPlannerQuery.h>
#include <ros/ros.h>

namespace supplementary
{
	class SystemConfig;
}

namespace msl
{
/**
 * Interface for invoking the path planner
 */
class MSLWorldModel;
class PathProxy
{
  public:
    PathProxy();
    virtual ~PathProxy();
    /**
     * Get ego direction form path planner
     * @param egoTarget shared_ptr<geometry::CNPoint2D>
     * @param eval shared_ptr<PathEvaluator>
     * @param additionalPoints shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> point to add as artificial obstacles to the Voronoi Diagram
     * @return shared_ptr<geometry::CNPoint2D> containing first waypoint of the calculated path
     */
    shared_ptr<geometry::CNPoint2D> getEgoDirection(shared_ptr<geometry::CNPoint2D> egoTarget, shared_ptr<IPathEvaluator> pathEvaluator,
                                                    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = nullptr);

    /**
     * Get ego direction form path planner
     * @param egoTarget shared_ptr<geometry::CNPoint2D>
     * @param eval shared_ptr<PathEvaluator>
     * @param query shared_ptr<PathPlannerQuery> encalsulates information given to the path planner
     * @return shared_ptr<geometry::CNPoint2D> containing first waypoint of the calculated path
     */
    shared_ptr<geometry::CNPoint2D> getEgoDirection(shared_ptr<geometry::CNPoint2D> egoTarget, shared_ptr<IPathEvaluator> pathEvaluator,
                                                    shared_ptr<PathPlannerQuery> query);

    /**
     * Get the path proxy instacne
     */
    static PathProxy *getInstance();
    /**
     * Send debug msg with pathplanner path
     * @param path shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
     */
    void sendPathPlannerMsg(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path);

    /**
     * Send debug msg with voroni infos
     * @param voronoi shared_ptr<VoronoiNet>
     */
    void sendVoronoiNetMsg(shared_ptr<VoronoiNet> voronoi);

    /**
     * Calculates cropped voronoi for debug msg
     * @param voronoi shared_ptr<VoronoiNet>
     */
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> calculateCroppedVoronoi(shared_ptr<VoronoiNet> voronoi);

  private:
    shared_ptr<geometry::CNPoint2D> lastPathTarget;
    MSLWorldModel *wm;
    ros::NodeHandle n;
    ros::Publisher pathPub;
    ros::Publisher voroniPub;
    supplementary::SystemConfig *sc;
    bool pathPlannerDebug;
    bool ignorePathsInsideRobot;

    /**
     * Calculates possible shortcuts in the path
     * @param path shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path returned by the Path Planner
     * @param ownPos shared_ptr<geometry::CNPosition>
     * @param net shared_ptr<VoronoiNet>
     * @return shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path conaining shortcuts
     */
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> applyShortcut(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path,
                                                                      shared_ptr<geometry::CNPosition> ownPos, shared_ptr<VoronoiNet> net);
};

} /* namespace msl */

