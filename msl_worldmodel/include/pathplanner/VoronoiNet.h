#pragma once

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

#include "MSLEnums.h"
#include "MSLFootballField.h"
#include "container/CNPoint2D.h"
#include "container/CNRobot.h"
#include "pathplanner/SearchNode.h"
#include "pathplanner/evaluator/IPathEvaluator.h"

namespace supplementary
{
class SystemConfig;
}

using namespace std;

namespace msl
{

namespace robot
{
	class IntRobotID;
}

/**
 * Class containing a CGAL Voronoi Diagram and its current status
 */
class PathEvaluator;
class MSLWorldModel;
class VoronoiNet
{
  public:
    VoronoiNet(MSLWorldModel *wm);
    VoronoiNet(shared_ptr<VoronoiNet> net);
    virtual ~VoronoiNet();
    bool ownPosAvail;
    /**
     * Generates a VoronoiDiagram and inserts given points
     * @param points vector<shared_ptr<geometry::CNPoint2D>>
     * @return shared_ptr<VoronoiDiagram>
     */
    void generateVoronoiDiagram(bool ownPosAvail);
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
     * @param point shared_ptr<geometry::CNPoint2D>
     * @return shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
     */
    shared_ptr<vector<shared_ptr<Vertex>>> getVerticesOfFace(shared_ptr<geometry::CNPoint2D> point);

    /**
     * Inserts sites into the Voronoi Diagram
     * @param points vector<Site_2>
     */
    void insertPoints(vector<Site_2> points);

    /**
     * Insert additional points into the Voronoi Diagram
     * @param points shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
     * @param type EntityType type of obstacle to insert
     */
    void insertAdditionalPoints(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points, EntityType type);

    /**
     * Deletes sites from Voronoi Net and clears pointRobotKindMapping
     */
    void clearVoronoiNet();

    /**
     * Gets the voronoi net
     * @return shared_ptr<VoronoiDiagram>
     */
    shared_ptr<VoronoiDiagram> getVoronoi();
    /**
     * Sets the voronoi net
     * @param voronoi shared_ptr<VoronoiDiagram>
     */
    void setVoronoi(shared_ptr<VoronoiDiagram> voronoi);
    /**
     * Find the face in which the point is situated
     * @param point VoronoiDiagram::Point_2
     * @return shared_ptr<VoronoiDiagram::Site_2>
     */
    shared_ptr<VoronoiDiagram::Site_2> getSiteOfFace(VoronoiDiagram::Point_2 point);

    /**
     * Return the obstacle positions
     * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > >
     */
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getObstaclePositions();

    /**
     * Return the Opponent positions
     * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > >
     */
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getOpponentPositions();

    /**
     * Return the site positions
     * @return shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > >
     */
    //		shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > getSitePositions();

    /**
     * Allo Obstacles for constructing this Voronoi Diagram
     */
    shared_ptr<vector<shared_ptr<geometry::CNRobot>>> getAlloClusteredObsWithMe();

    /**
     * Artificial Obstacles for construction this Voronoi Diagram
     */
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getArtificialObstacles();

    /**
     * Additional Obstacles for construction this Voronoi Diagram
     */
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getAdditionalObstacles();

    /**
     * Return Vertices of teammates Voronoi Face
     * @param teamMateId int
     * @return shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
     */
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getTeamMateVerticesCNPoint2D(const msl::robot::IntRobotID* teamMateId);
    /**
     * Removes given sites from Voronoi Diagram
     * @param shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> sites to remove
     */
    void removeSites(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> sites);

    /**
     * Removes given sites from Voronoi Diagram
     * @param shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> sites to remove
     */
    void removeSites(shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> sites);

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
     * @param shared_ptr<geometry::CNPoint2D> centerPoint
     * @param double radius
     */
    void blockCircle(shared_ptr<geometry::CNPoint2D> centerPoint, double radius);

    /**
     * Block opponent penalty area
     * @param shared_ptr<geometry::CNPoint2D> upLeftCorner
     * @param shared_ptr<geometry::CNPoint2D> lowRightCorner
     */
    void blockRectangle(shared_ptr<geometry::CNPoint2D> upLeftCorner, shared_ptr<geometry::CNPoint2D> lowRightCorner);

    /**
     * Checks type of Voronoi Site (artificial, team, opponent)
     * @return int type of Site
     */
    int getTypeOfSite(Site_2 site);

  private:
    /**
     * Checks if a SearchNode is part of a vector
     * @param shared_ptr<vector<shared_ptr<SearchNode>>>
     * @param shared_ptr<SearchNode>
     * @return bool ture if SearchNode is in
     */
    bool contains(shared_ptr<vector<shared_ptr<SearchNode>>> vector, shared_ptr<SearchNode> vertex);

  protected:
    Kernel kernel;
    DelaunayTriangulation delaunayTriangulation;
    DelaunayAdaptionTraits delaunayTraits;
    DelaunayAdaptionPolicy delaunayPolicy;
    shared_ptr<VoronoiDiagram> voronoi;
    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;
    mutex netMutex;
    /**
     * team = robot id, obstacle = -1, artificial obstacle = -2
     */
    // map<shared_ptr<geometry::CNPoint2D>, int> pointRobotKindMapping;
    map<Site_2, int> pointRobotKindMapping;

    // DATA for constructing this voronoi net
    shared_ptr<vector<shared_ptr<geometry::CNRobot>>> alloClusteredObsWithMe;
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> artificialObstacles;
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalObstacles;
};

} /* namespace msl */
