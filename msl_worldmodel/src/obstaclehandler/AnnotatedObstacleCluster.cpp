/*
 * AnnotatedObstacleCluster.cpp
 *
 *  Created on: Feb 11, 2016
 *      Author: Stefan Jakob
 */

#include "obstaclehandler/AnnotatedObstacleCluster.h"
#include "MSLEnums.h"
#include "obstaclehandler/AnnotatedObstacleClusterPool.h"
#include <sstream>

namespace msl
{

AnnotatedObstacleCluster::AnnotatedObstacleCluster()
{
    this->velX = 0;
    this->velY = 0;
    this->ident = 0;
    this->x = 0;
    this->y = 0;
    this->radius = 0.0;
    this->linearSumX = 0;
    this->linearSumY = 0;
    this->squareSum = 0;
    this->angle = 0;
    this->rotation = 0;
    this->certainty = 0;
    this->numObs = 0;
    this->supporter = make_shared<vector<int>>();
    this->opposer = make_shared<vector<int>>();

    //		this->vEdges = new ArrayList<VEdge>();
    //		this->vNodes = new ArrayList<VNode>();
}

AnnotatedObstacleCluster::~AnnotatedObstacleCluster()
{
}

void AnnotatedObstacleCluster::clear()
{
    this->ident = 0;
    this->x = 0;
    this->y = 0;
    this->radius = 0.0;
    this->linearSumX = 0;
    this->linearSumY = 0;
    this->squareSum = 0;

    this->numObs = 0;
    this->supporter->clear();
    this->opposer->clear();
    //		this.vEdges.Clear ();
    //		this.vNodes.Clear ();
}

void AnnotatedObstacleCluster::init(int x, int y, double radius, int ident, int supId)
{
    this->init(x, y, 0.0, radius, 0, 0, 0.0, 0.0, ident, supId);
}

void AnnotatedObstacleCluster::init(int x, int y, double radius, int velX, int velY, int ident, int supId)
{
    this->init(x, y, 0.0, radius, velX, velY, 0.0, 0.0, ident, supId);
}

void AnnotatedObstacleCluster::init(int x, int y, double angle, double radius, int velX, int velY, double rotation, double certainty, int ident, int supId)
{
    this->x = x;
    this->y = y;
    this->angle = angle;
    this->radius = radius;
    this->velX = velX;
    this->velY = velY;
    this->rotation = rotation;
    this->certainty = certainty;
    this->ident = ident;
    this->numObs = 1;

    // set initial linear and square sum
    this->linearSumX = x;
    this->linearSumY = y;
    this->squareSum = x * x + y * y;

    // add the id of the new supporter
    if (supId != -1) // for artificial obstacles
    {
        this->supporter->push_back(supId);
    }
}

double AnnotatedObstacleCluster::getVariance()
{
    double numObs = (double)this->numObs;
    double avgX = this->linearSumX / numObs;
    double avgY = this->linearSumY / numObs;
    return (this->squareSum + numObs * ((avgX * avgX) + (avgY * avgY)) - 2 * ((avgX * this->linearSumX) + (avgY * this->linearSumY))) / numObs;
}

void AnnotatedObstacleCluster::add(shared_ptr<AnnotatedObstacleCluster> obs)
{
    // calculate the new centroid
    this->x = (this->x * this->numObs + obs->x);
    this->y = (this->y * this->numObs + obs->y);
    this->numObs++;
    this->x /= this->numObs;
    this->y /= this->numObs;

    // update the linear and square sum
    this->addToLinearSum(obs);
    this->addToSquareSum(obs);

    // add the id of the new supporter
    for (int i = 0; i < supporter->size(); i++)
    {
        this->supporter->push_back(obs->supporter->at(i));
    }
}

bool AnnotatedObstacleCluster::checkAndMerge(AnnotatedObstacleCluster *cluster, double varianceThreshold)
{
    // variables as if both clusters are merged
    double tmpNumObs = (double)this->numObs + cluster->numObs;
    int tmplinearSumX = this->linearSumX + cluster->linearSumX;
    int tmplinearSumY = this->linearSumY + cluster->linearSumY;
    int tmpSquareSum = this->squareSum + cluster->squareSum;
    // calculate variance
    double avgX = tmplinearSumX / tmpNumObs;
    double avgY = tmplinearSumY / tmpNumObs;
    double tmpVariance = (tmpSquareSum + tmpNumObs * ((avgX * avgX) + (avgY * avgY)) - 2 * ((avgX * tmplinearSumX) + (avgY * tmplinearSumY))) / tmpNumObs;

    if (tmpVariance > varianceThreshold)
    {
        // two cluster which break the VARIANCE_THRESHOLD -> the complete clustering is finished
        return false;
    }
    else
    {
        // merge these two clusters
        this->numObs = (int)tmpNumObs;
        this->linearSumX = tmplinearSumX;
        this->linearSumY = tmplinearSumY;
        this->squareSum = tmpSquareSum;
        this->x = (int)avgX;
        this->y = (int)avgY;
        int range = cluster->supporter->size();
        for (int i = 0; i < range; i++)
        {
            this->supporter->push_back(cluster->supporter->at(i));
        }
        if (cluster->ident != EntityType::Opponent)
        {
            // update this.ident with ident of other cluster
            this->ident = cluster->ident;
            this->radius = cluster->radius;
            this->angle = cluster->angle;
            this->velX = cluster->velX;
            this->velY = cluster->velY;
            this->certainty = cluster->certainty;
            this->rotation = cluster->rotation;
        }
        return true;
    }
}

void AnnotatedObstacleCluster::remove(shared_ptr<AnnotatedObstacleCluster> obs)
{
    // calculate the new centroid
    this->x = this->x * this->numObs - obs->x;
    this->y = this->y * this->numObs - obs->y;
    this->numObs--;
    this->x /= this->numObs;
    this->y /= this->numObs;

    // update the linear and square sum
    this->subFromLinearSum(obs);
    this->subFromSquareSum(obs);
}

double AnnotatedObstacleCluster::distanceTo(AnnotatedObstacleCluster *aoc)
{
    return sqrt(pow(this->x - aoc->x, 2) + pow(this->y - aoc->y, 2));
}

double AnnotatedObstacleCluster::distanceTo(shared_ptr<geometry::CNPosition> pos)
{
    return sqrt(pow(this->x - pos->x, 2) + pow(this->y - pos->y, 2));
}

double AnnotatedObstacleCluster::distanceTo(shared_ptr<geometry::CNPoint2D> p)
{
    return sqrt(pow(this->x - p->x, 2) + pow(this->y - p->y, 2));
}

bool AnnotatedObstacleCluster::compareTo(AnnotatedObstacleCluster *first, AnnotatedObstacleCluster *second)
{
    // first: try to sort by x coordinate
    if (second->x > first->x)
    {
        return true;
    }
    else if (second->x < first->x)
    {
        return false;
    }
    else
    {
        // second: try to sort by y coordinate
        if (second->y > first->y)
        {
            return true;
        }
        else if (second->y < first->y)
        {
            return false;
        }
        else
        {
            return false;
            // TODO sort by hash code
            //				if (other->GetHashCode() > this->GetHashCode())
            //				{
            //					return 1;
            //				}
            //				else if (other->GetHashCode() < this->GetHashCode())
            //				{
            //					return -1;
            //				}
            //				else
            //				{
            //					cout << "OC: Two times the same object:\nX1: "
            //					                  << this->x << " Y1: " << this->y
            //					                  << "\nX2: " << other->x << " Y2: " << other->y << endl;
            //					return 0;
            //				}
        }
    }
}

bool AnnotatedObstacleCluster::equals(shared_ptr<AnnotatedObstacleCluster> cl)
{
    if (cl->x == this->x && cl->y == this->y)
    {
        this->radius = max(this->radius, cl->radius);
        cl->radius = this->radius;
        //				cout << "AOC: found equal!" << endl;
        return true;
    }
    return false;
}

string AnnotatedObstacleCluster::toString()
{
    stringstream ss;
    ss << "Ident: " << this->ident << "\nCentroid X: " << this->x << " Y: " << this->y << " Radius: " << this->radius << "\nPoints: " << this->numObs;

    ss << "\nSupporters: ";
    for (int robot : *supporter)
    {
        ss << robot << " ";
    }

    ss << "\nNegSupporters: ";
    for (int robot : *opposer)
    {
        ss << robot << " ";
    }

    ss << "\nVariance: " << this->getVariance();

    return ss.str();
}

void AnnotatedObstacleCluster::addToLinearSum(shared_ptr<AnnotatedObstacleCluster> aoc)
{
    this->linearSumX += aoc->x;
    this->linearSumY += aoc->y;
}

void AnnotatedObstacleCluster::addToSquareSum(shared_ptr<AnnotatedObstacleCluster> aoc)
{
    this->squareSum += (aoc->x * aoc->x + aoc->y * aoc->y);
}

void AnnotatedObstacleCluster::subFromLinearSum(shared_ptr<AnnotatedObstacleCluster> aoc)
{
    this->linearSumX -= aoc->x;
    this->linearSumY -= aoc->y;
}

// TODO Seg fault
AnnotatedObstacleCluster *AnnotatedObstacleCluster::getNew(AnnotatedObstacleClusterPool *aocp)
{
    if (aocp->curIndex >= aocp->maxCount)
    {
        cerr << "max AOC count reached!" << endl;
    }
    AnnotatedObstacleCluster *ret = aocp->daAOCs[aocp->curIndex++];
    ret->clear();
    return ret;
}

void AnnotatedObstacleCluster::subFromSquareSum(shared_ptr<AnnotatedObstacleCluster> aoc)
{
    this->squareSum -= (aoc->x * aoc->x + aoc->y * aoc->y);
}

} /* namespace msl */
