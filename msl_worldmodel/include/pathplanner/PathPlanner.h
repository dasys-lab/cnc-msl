#pragma once

#include "MSLEnums.h"
#include "MSLFootballField.h"
#include "pathplanner/SearchNode.h"
#include "pathplanner/VoronoiNet.h"
#include "pathplanner/evaluator/PathEvaluator.h"

#include <SystemConfig.h>
#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPositionAllo.h>
#include <nonstd/optional.hpp>

#include <msl_msgs/CorridorCheck.h>
#include <msl_msgs/Point2dInfo.h>
#include <msl_sensor_msgs/WorldModelData.h>
#include <ros/ros.h>

// includes for CGAL
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Voronoi_diagram_2.h>

/**
 * typedefs to short CGAL's class names
 e.g.CGAL::Voronoi_diagram_2<CGAL::Delaunay_triangulation_2<CGAL::Exact_predicates_inexact_constructions_kernel>,
 CGAL::Delaunay_triangulation_adaptation_traits_2<CGAL::Delaunay_triangulation_2<CGAL::Exact_predicates_inexact_constructions_kernel>>,
 CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<CGAL::Delaunay_triangulation_2<CGAL::Exact_predicates_inexact_constructions_kernel>>>
 to VoronoiDiagram
 */
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DelaunayTriangulation> DelaunayAdaptionTraits;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DelaunayTriangulation> DelaunayAdaptionPolicy;
typedef CGAL::Voronoi_diagram_2<DelaunayTriangulation, DelaunayAdaptionTraits, DelaunayAdaptionPolicy> VoronoiDiagram;
typedef DelaunayAdaptionTraits::Point_2 Point_2;
typedef DelaunayAdaptionTraits::Site_2 Site_2;
typedef VoronoiDiagram::Vertex Vertex;

#include <algorithm>
#include <limits>
#include <math.h>
#include <memory>
#include <mutex>
#include <vector>

/**
 * Calculates path from given startpoint to a given goalpoint, while avoiding Obstacles.
 */
namespace msl
{

class MSLWorldModel;
class VoronoiNet;
class PathEvaluator;
class PathPlanner
{
  public:
    PathPlanner(MSLWorldModel *wm, int count);
    virtual ~PathPlanner();

    /**
     * Method to plan the Path based on A*-Algorithm
     * @param voronoi shared_ptr<VoronoiNet> Voronoi Net used druing this planning call
     * @param ownPos Point_2 Start point
     * @param goal Point_2 Destination Point
     * @return shared_ptr<vector<shared_ptr<Point_2>>> Path leading to the goal. Points represent Voronoi Vertices.
     */
    std::shared_ptr<std::vector<geometry::CNPointAllo>> plan(const VoronoiNet &voronoi, geometry::CNPointAllo startPos,
                                                             geometry::CNPointAllo goal, const IPathEvaluator &eval);

    /**
     * Generate new Voronoi Diagram with artificial Surrounding and own position if available.
     */
    void prepareVoronoiDiagram();
    /**
     * Get all saved Voronoi Diagrams
     * @return vector<shared_ptr<VoronoiNet>>
     */
    std::vector<std::shared_ptr<const VoronoiNet>> getVoronoiNets() const;
    /**
     * Get latest accessible Voronoi Diagram
     * @return shared_ptr<VoronoiNet>
     */
    std::shared_ptr<const VoronoiNet> getCurrentVoronoiNet() const;

    /**
     * Gets robot diameter in mm
     * @return double
     */
    double getRobotRadius() const;
    /**
     * Gets pathDeviationWeight (PathPlanner.conf) used to punish deviation of two paths
     * @return double
     */
    double getPathDeviationWeight() const;
    /**
     * Gets dribbleAngleTolerance (PathPlanner.conf) used define angle tolerance for dribbling
     * @return double
     */
    double getDribbleAngleTolerance() const;
    /**
     * Gets dribbleRotationWeight (PathPlanner.conf) used to punish high angle deviation between two paths
     * @return double
     */
    double getDribbleRotationWeight() const;
    /**
     * Gets additionalCorridorWidth (PathPlanner.conf) used to increase corridor check size
     * @return double
     */
    double getAdditionalCorridorWidth() const;
    /**
     * Inserts a searchnode into a sorted vector
     */
    static void insert(std::vector<SearchNode> &vect, std::shared_ptr<SearchNode> currentNode);
    /**
     * Gets the base voronoi net with artificial field surrounding
     * @return shared_ptr<VoronoiNet>
     */
    std::shared_ptr<const VoronoiNet> getArtificialObjectNet() const;

    /**
     * Gets the base voronoi net with artificial field surrounding as mutable
     * @return shared_ptr<VoronoiNet>
     */
    std::shared_ptr<VoronoiNet> getArtificialObjectNet();
    /**
     * Gets artificial obstacles surrounding the field
     * @return shared_ptr<vector<shared_ptr<geometry::CNPointAllo>>>
     */
    std::shared_ptr<const std::vector<geometry::CNPointAllo>> getArtificialFieldSurroundingObs() const;
    /**
     * Gets last returned path
     * @return shared_ptr<vector<shared_ptr<geometry::CNPointAllo>>>
     */
    std::shared_ptr<const std::vector<geometry::CNPointAllo>> getLastPath() const;
    /**
     * Gets last destination point to check if two path lead to the same goal
     * @return shared_ptr<geometry::CNPointAllo>
     */
    nonstd::optional<geometry::CNPointAllo> getLastTarget() const;

    /**
     * Checks if a Voronoi Edge is part of the path
     * @param vector shared_ptr<vector<shared_ptr<SearchNode> > > vector vector to check
     * @param edge VoronoiDiagram::Halfedge_around_vertex_circulator edge to check
     * @return bool true if the edge is part of the path
     */
    static bool contains(const std::vector<SearchNode> &vector,
                         VoronoiDiagram::Halfedge_around_vertex_circulator edge);

    /**
     * Checks if the Voronoi Edge should not be expanded
     * @param incidentHalfEdge VoronoiDiagram::Halfedge_around_vertex_circulator edge to check if admissible
     * @param startPos shared_ptr<geometry::CNPointAllo> start position of the path
     * @param voronoi shared_ptr<VoronoiNet> Voronoi Diagram to use
     * @return bool true if admissible
     */
    bool isAdmissableEdge(VoronoiDiagram::Halfedge_around_vertex_circulator incidentHalfEdge,
                          geometry::CNPointAllo startPos, const VoronoiNet& voronoi) const;
    /**
     * Expands a SearchNode used in the A*-Algorithm
     * @param currentNode shared_ptr<SearchNode>
     * @param open shared_ptr<vector<shared_ptr<SearchNode>>> searchnodes to check
     * @param closed shared_ptr<vector<shared_ptr<SearchNode>>> searchnodes checked
     * @param startPos shared_ptr<geometry::CNPointAllo>
     * @param goal shared_ptr<geometry::CNPointAllo>
     * @param eval shared_ptr<PathEvaluator>
     */
    void expandNode(std::shared_ptr<SearchNode> currentNode, std::vector<SearchNode> &open,
                    const std::vector<SearchNode> &closed, geometry::CNPointAllo startPos,
                    geometry::CNPointAllo goal, const IPathEvaluator &eval,
                    const VoronoiNet &voronoi);

    /**
     * Checks if an Opponent is close to the ball
     * @param voronoi shared_ptr<VoronoiNet>
     * @param currentPos shared_ptr<geometry::CNPointAllo>
     * @param goal shared_ptr<geometry::CNPointAllo>
     * @param obstaclePoint shared_ptr<geometry::CNPointAllo>
     * @return bool true if there is an Opponent close to the ball
     */
    bool closeOppToBallCheck(const VoronoiNet &voronoi, geometry::CNPointAllo currentPos,
                             geometry::CNPointAllo goal, geometry::CNPointAllo obstaclePoint);

    /**
     * Checks if there is an obstacle inside the corridor
     * @param voronoi shared_ptr<VoronoiNet>
     * @param currentPos shared_ptr<geometry::CNPointAllo>
     * @param goal shared_ptr<geometry::CNPointAllo>
     * @param obstaclePoint shared_ptr<geometry::CNPointAllo>
     * @return bool true if an obstacle is inside the corridor
     */
    bool corridorCheck(geometry::CNPointAllo currentPos, geometry::CNPointAllo goal,
                       geometry::CNPointAllo obstaclePoint, double obstacleRadius = 0.0);

    /**
     * Checks if there is an obstacle inside the corridor (smaller corridor size for ball)
     * @param voronoi shared_ptr<VoronoiNet>VoronoiNet*
     * @param currentPos shared_ptr<geometry::CNPointAllo>
     * @param goal shared_ptr<geometry::CNPointAllo>
     * @param obstaclePoint shared_ptr<geometry::CNPointAllo>
     * @return bool true if an obstacle is inside corridor
     */
    bool corridorCheckBall(geometry::CNPointAllo CNPointAllo, geometry::CNPointAllo goal,
                           geometry::CNPointAllo obstaclePoint, double obstacleRadius = 0.0);

  private:
    /**
     * Initializes artificial objects and insert them into artificial object net
     */
    void initializeArtificialObstacles();
    /**
     * Compares two search nodes
     * @param first shared_ptr<SearchNode>
     * @param second shared_ptr<SearchNode>
     * @return bool true if sum of cost and heuristic of first niode is smaller
     */
    static bool compare(std::shared_ptr<const SearchNode> first, std::shared_ptr<const SearchNode> second);
    /**
     * Checks if vertices of goal face are reached
     * @param closestVerticesToGoal shared_ptr<vector<shared_ptr<Vertex> > > vertices of Voronoi face containing the
     * goal point
     * @param currentNode shared_ptr<SearchNode>
     * @return bool true if goal Voronoi Face reached
     */
    bool checkGoalVerticesReached(const std::vector<Vertex> &closestVerticesToGoal,
                                  SearchNode &currentNode);
    /**
     * Helping method to debug the corridor check (send Corridor check as msg)
     * @param points vector<shared_ptr<geometry::CNPointAllo>> vertex point of the corridor check
     */
    void sendCorridorCheck(std::vector<geometry::CNPointAllo> points);

    /**
     * Calculate distance between point and Voronoi Vertex
     * @param v1 shared_ptr<geometry::CNPointAllo>
     * @param v2 shared_ptr<Vertex>
     * @return double
     */
    double distanceTo(geometry::CNPointAllo v1, Vertex v2);

    /**
     * A*-Search on a VoronoiDiagram
     * @param voronoi shared_ptr<VoronoiNet>
     * @param startPos Point_2
     * @param goal Point_2
     * @return shared_ptr<vector<shared_ptr<Point_2>>> resulting optimal path regarding the Pathevaluator
     */
    std::shared_ptr<std::vector<geometry::CNPointAllo>>
    aStarSearch(const VoronoiNet &voronoi, geometry::CNPointAllo startPos, geometry::CNPointAllo goal,
                const IPathEvaluator &pathEvaluator);

  protected:
    /**
     * WorldModel pointer
     */
    MSLWorldModel *wm;
    /**
     * Defines which Voronoi Diagram to use
     */
    int currentVoronoiPos;
    /**
     * SystemConfig pointer
     */
    supplementary::SystemConfig *sc;
    /**
     * Guards the Voronoi Diagram vector
     */
    mutable mutex voronoiMutex;
    std::vector<std::shared_ptr<VoronoiNet>> voronoiDiagrams;
    double robotRadius;
    std::shared_ptr<VoronoiNet> artificialObjectNet;
    double pathDeviationWeight;
    double dribble_rotationWeight;
    double dribble_angleTolerance;
    double minEdgeWidth;
    double corridorWidthDivisor;
    bool pathPlannerDebug;
    double marginToBlockedArea;
    double additionalCorridorWidth;
    nonstd::optional<SearchNode> lastClosestNode;
    double snapDistance;
    double additionalBallCorridorWidth;
    double corridorWidthDivisorBall;

    ros::NodeHandle n;
    ros::Publisher corridorPub;
    nonstd::optional<geometry::CNPointAllo> lastClosestPointToBlock;
    nonstd::optional<geometry::CNPointAllo> lastTarget;
    std::shared_ptr<std::vector<geometry::CNPointAllo>> lastPath;
    /**
     * Check if the goal vertices are reached and if there is a corridor leading to the goal
     * @param voronoi shared_ptr<VoronoiNet>
     * @param currentNode shared_ptr<SearchNode>
     * @param closestVerticesToGoal shared_ptr<vector<shared_ptr<Vertex>>> vertices of Voronoi face containing the goal
     * point
     * @param goal shared_ptr<geometry::CNPointAllo>
     * @return bool true if goal is reachable
     */
    bool checkGoalReachable(const VoronoiNet &voronoi, SearchNode &currentNode,
                            const std::vector<Vertex> &closestVerticesToGoal,
                            geometry::CNPointAllo goal);
};
} /* namespace msl */
