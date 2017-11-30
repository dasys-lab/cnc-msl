#pragma once

#include "MSLEnums.h"
#include "MSLFootballField.h"
#include "pathplanner/SearchNode.h"
#include "pathplanner/evaluator/IPathEvaluator.h"

#include <cnc_geometry/CNPointAllo.h>
#include <obstaclehandler/CNRobotAllo.h>
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
typedef DelaunayAdaptionTraits::Point_2 Point_2;
typedef DelaunayAdaptionTraits::Site_2 Site_2;
typedef VoronoiDiagram::Vertex Vertex;

#include <iostream>
#include <sstream>
#include <vector>

namespace supplementary
{
class SystemConfig;
}

namespace msl
{

/**
 * Class containing a CGAL Voronoi Diagram and its current status
 */
class PathEvaluator;
class MSLWorldModel;
class VoronoiNet
{
    // TODO: cleanup and reqork documentation
  public:
    VoronoiNet(MSLWorldModel *wm);
    VoronoiNet(std::shared_ptr<VoronoiNet> net);
    virtual ~VoronoiNet();
    bool ownPosAvail;
    /**
     * Generates a VoronoiDiagram.
     * @param ownPosAvail tells the diagram if the own pos is available
     */
    void generateVoronoiDiagram(bool ownPosAvail);
    /**
     * Gets the SearchNode with lowest dist to goal
     * @param open std::shared_ptr<std::vector<std::shared_ptr<SearchNode>>>
     * @return std::shared_ptr<SearchNode>
     */
    //		std::shared_ptr<SearchNode> getMin(std::shared_ptr<std::vector<std::shared_ptr<SearchNode>>> open);
    /**
     * Gets the closest vertex to a given point
     * @param ownPos std::shared_ptr<geometry::CNPoint2D>
     * @return std::shared_ptr<VoronoiDiagram::Vertex>
     */
    //		std::shared_ptr<VoronoiDiagram::Vertex> findClosestVertexToOwnPos(std::shared_ptr<geometry::CNPoint2D> ownPos);

    /**
     * Return the sites near an egde defined by 2 points
     * @param v1 std::shared_ptr<geometry::CNPoint2D>
     * @param v2 std::shared_ptr<geometry::CNPoint2D>
     * @return pair<pair<std::shared_ptr<geometry::CNPoint2D>, int>, pair<std::shared_ptr<geometry::CNPoint2D>, int>>
     */
    // pair<pair<std::shared_ptr<geometry::CNPoint2D>, int>, pair<std::shared_ptr<geometry::CNPoint2D>, int>> getSitesNextToHalfEdge(std::shared_ptr<Vertex> v1,
    // std::shared_ptr<Vertex> v2);
    /**
     * Print the voronoi diagrams sites
     */
    void printSites();
    /**
     * Print the voronoi diagrams vertices
     */
    void printVertices();
    /**
     * To string contains all parts of the Voronoi Diagram
     */
    string toString();

    /**
     * Locates face of point and returns vertices
     * @param point geometry::CNPoint2D
     * @return std::shared_ptr<std::vector<std::shared_ptr<geometry::CNPoint2D>>>
     */
    std::shared_ptr<std::vector<Vertex>> getVerticesOfFace(geometry::CNPointAllo point) const;

    /**
     * Inserts sites into the Voronoi Diagram
     * @param points std::vector<Site_2>
     */
    void insertPoints(std::vector<Site_2> points);

    /**
     * Insert additional points into the Voronoi Diagram
     * @param points std::shared_ptr<std::vector<std::shared_ptr<geometry::CNPoint2D>>>
     * @param type EntityType type of obstacle to insert
     */
    void insertAdditionalPoints(const std::vector<geometry::CNPointAllo>  &points, EntityType type);

    /**
     * Deletes sites from Voronoi Net and clears pointRobotKindMapping
     */
    void clearVoronoiNet();

    /**
    * Gets the voronoi net as mutable
    * @return std::shared_ptr<VoronoiDiagram>
    */
    std::shared_ptr<VoronoiDiagram> getVoronoi();

    /**
     * Gets the voronoi net
     * @return std::shared_ptr<VoronoiDiagram>
     */
    std::shared_ptr<const VoronoiDiagram> getVoronoi() const;
    /**
     * Sets the voronoi net
     * @param voronoi std::shared_ptr<VoronoiDiagram>
     */
    void setVoronoi(std::shared_ptr<VoronoiDiagram> voronoi);
    /**
     * Find the face in which the point is situated
     * @param point VoronoiDiagram::Point_2
     * @return std::shared_ptr<VoronoiDiagram::Site_2>
     */
    nonstd::optional<VoronoiDiagram::Site_2> getSiteOfFace(VoronoiDiagram::Point_2 point) const;

    /**
     * Check if an edge belongs to face of given point
     * @param pos std::shared_ptr<geometry::CNPoint2D>
     * @param currentNode std::shared_ptr<SearchNode>
     * @param nextNode std::shared_ptr<SearchNode>
     * @return bool
     */
    //		bool isOwnCellEdge(std::shared_ptr<geometry::CNPoint2D> pos, std::shared_ptr<SearchNode> currentNode, std::shared_ptr<SearchNode> nextNode);

    /**
     * Return the teammate positions
     * @return std::shared_ptr<std::vector<pair<std::shared_ptr<geometry::CNPoint2D>, int> > >
     */
    // std::shared_ptr<std::vector<pair<std::shared_ptr<geometry::CNPoint2D>, int> > > getTeamMatePositions();

    /**
     * Return the obstacle positions
     * @return std::shared_ptr<std::vector<pair<std::shared_ptr<geometry::CNPoint2D>, int> > >
     */
    std::shared_ptr<std::vector<geometry::CNPointAllo>> getObstaclePositions();

    /**
     * Return the Opponent positions
     * @return std::shared_ptr<std::vector<pair<std::shared_ptr<geometry::CNPoint2D>, int> > >
     */
    std::shared_ptr<std::vector<geometry::CNPointAllo>> getOpponentPositions();

    /**
     * Return the site positions
     * @return std::shared_ptr<std::vector<pair<std::shared_ptr<geometry::CNPoint2D>, int> > >
     */
    //		std::shared_ptr<std::vector<pair<std::shared_ptr<geometry::CNPoint2D>, int> > > getSitePositions();

    /**
     * Allo Obstacles for constructing this Voronoi Diagram
     */
    std::shared_ptr<const std::vector<CNRobotAllo>> getAlloClusteredObsWithMe() const;

    /**
     * Artificial Obstacles for construction this Voronoi Diagram
     */
    std::shared_ptr<const std::vector<geometry::CNPointAllo>> getArtificialObstacles() const;

    /**
     * Additional Obstacles for construction this Voronoi Diagram
     */
    std::shared_ptr<const std::vector<geometry::CNPointAllo>> getAdditionalObstacles() const;

    /**
     * Return vertices teammates voronoi face
     * @param teamMateId int
     * @return std::shared_ptr<std::vector<std::shared_ptr<geometry::CNPoint2D>>>
     */
    //		std::shared_ptr<std::vector<std::shared_ptr<Vertex>>> getTeamMateVertices(int teamMateId);

    /**
     * Return Vertices of teammates Voronoi Face
     * @param teamMateId int
     * @return std::shared_ptr<std::vector<std::shared_ptr<geometry::CNPoint2D>>>
     */
    std::shared_ptr<std::vector<geometry::CNPointAllo>> getTeamMateVerticesCNPoint2D(int teamMateId);
    /**
     * Removes given sites from Voronoi Diagram
     * @param std::shared_ptr<std::vector<std::shared_ptr<geometry::CNPoint2D>>> sites to remove
     */
    void removeSites(std::shared_ptr<const std::vector<geometry::CNPointAllo>> sites);

    /**
     * Removes given sites from Voronoi Diagram
     * @param std::shared_ptr<std::vector<pair<std::shared_ptr<geometry::CNPoint2D>, int>>> sites to remove
     */
    void removeSites(std::shared_ptr<const std::vector<std::pair<geometry::CNPointAllo, int>>> sites);

    /**
     * Block opponent penalty area
     */
    void blockOppPenaltyArea();

    /**
     * Block opponent goal area
     */
    void blockOppGoalArea();

    /**
     * Block own penalty area
     */
    void blockOwnPenaltyArea();

    /**
     * Block own goal area
     */
    void blockOwnGoalArea();

    /**
     * Block 3 meters around the ball
     */
    void blockThreeMeterAroundBall();

    /**
     * Block circle shaped area
     * @param centerPoint center of blocking area
     * @param radius radius of blocking area
     */
    void blockCircle(geometry::CNPointAllo centerPoint, double radius);

    /**
     * Block opponent penalty area
     * @param upLeftCorner upper left corner
     * @param lowRightCorner lower right corner
     */
    void blockRectangle(geometry::CNPointAllo upLeftCorner, geometry::CNPointAllo lowRightCorner);

    /**
     * Checks type of Voronoi Site (artificial, team, opponent)
     * @return int type of Site
     */
    int getTypeOfSite(Site_2 site) const;

  private:
    /**
     * Checks if a SearchNode is part of a std::vector
     * @param std::shared_ptr<std::vector<std::shared_ptr<SearchNode>>>
     * @param std::shared_ptr<SearchNode>
     * @return bool ture if SearchNode is in
     */
    bool contains(std::shared_ptr<std::vector<SearchNode>> vector, SearchNode vertex);

  protected:
    Kernel kernel;
    DelaunayTriangulation delaunayTriangulation;
    DelaunayAdaptionTraits delaunayTraits;
    DelaunayAdaptionPolicy delaunayPolicy;
    std::shared_ptr<VoronoiDiagram> voronoi;
    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;
    mutex netMutex;
    /**
     * team = robot id, obstacle = -1, artificial obstacle = -2
     */
    // std::map<std::shared_ptr<geometry::CNPoint2D>, int> pointRobotKindMapping;
    std::map<Site_2, int> pointRobotKindMapping;

    // DATA for constructing this voronoi net
    std::shared_ptr<std::vector<CNRobotAllo>> alloClusteredObsWithMe;
    std::shared_ptr<std::vector<geometry::CNPointAllo>> artificialObstacles;
    std::shared_ptr<std::vector<geometry::CNPointAllo>> additionalObstacles;
};

} /* namespace msl */
