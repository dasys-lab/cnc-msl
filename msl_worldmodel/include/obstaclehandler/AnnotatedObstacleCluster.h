#pragma once

#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include <memory>
#include <vector>
using namespace std;

namespace msl
{

class AnnotatedObstacleClusterPool;
class AnnotatedObstacleCluster
{
  public:
    AnnotatedObstacleCluster();
    virtual ~AnnotatedObstacleCluster();
    /* identifier:
     * -10000 = global triangle points
     * -5000  = additional obstacles (inserted by behaviours)
     * -1     = opponent
     * x<-1   = artificial field approximation obstacle
     * 0      = undefined (like just constructed)
     * x>0    = teammate with id = x */
    int ident;

    // FOR VORONOI-DIAGRAMM
    //		ArrayList<VEdge> vEdges = null;
    //		ArrayList<VNode> vNodes = null;
    //		bool isInsideField;

    // obstacle specifications (including centroid coordinates for clustering):
    int x;
    int y;
    double angle; // just for teammates
    double radius;
    double certainty; // just for teammates
    int velX;
    int velY;
    double rotation; // just for teammates

    // FOR CLUSTERING
    int numObs = 0;
    // Linear Sum
    int linearSumX;
    int linearSumY;
    // Square Sum
    int squareSum;
    // All teammates which see one of these obstacles
    shared_ptr<vector<int>> supporter;
    // All teammates which should see one of these obstacles, but does not
    shared_ptr<vector<int>> opposer;
    void clear();
    void init(int x, int y, double radius, int ident, int supId);
    void init(int x, int y, double radius, int velX, int velY, int ident, int supId);
    void init(int x, int y, double angle, double radius, int velX, int velY, double rotation, double certainty, int ident, int supId);
    double getVariance();
    void add(shared_ptr<AnnotatedObstacleCluster> obs);
    bool checkAndMerge(AnnotatedObstacleCluster *cluster, double varianceThreshold);
    void remove(shared_ptr<AnnotatedObstacleCluster> obs);
    double distanceTo(AnnotatedObstacleCluster *aoc);
    double distanceTo(shared_ptr<geometry::CNPosition> pos);
    double distanceTo(shared_ptr<geometry::CNPoint2D> p);
    static bool compareTo(AnnotatedObstacleCluster *first, AnnotatedObstacleCluster *second);
    bool equals(shared_ptr<AnnotatedObstacleCluster> cl);
    static AnnotatedObstacleCluster *getNew(AnnotatedObstacleClusterPool *aocp);
    string toString();

  protected:
    void addToLinearSum(shared_ptr<AnnotatedObstacleCluster> aoc);
    void addToSquareSum(shared_ptr<AnnotatedObstacleCluster> aoc);
    void subFromLinearSum(shared_ptr<AnnotatedObstacleCluster> aoc);
    void subFromSquareSum(shared_ptr<AnnotatedObstacleCluster> aoc);
};

} /* namespace msl */
