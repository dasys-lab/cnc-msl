/*
 * SimpleCluster.cpp
 *
 *  Created on: Feb 11, 2016
 *      Author: Stefan Jakob
 */

#include "obstaclehandler/SimpleCluster.h"

namespace msl
{

SimpleCluster::SimpleCluster(shared_ptr<geometry::CNPoint2D> p)
{
    this->x = (int)(p->x + 0.5);
    this->y = (int)(p->y + 0.5);
    this->linearSumX = this->x;
    this->linearSumY = this->y;
    this->squareSum = this->x * this->x + this->y * this->y;
    this->numObs = 1;
}

SimpleCluster::~SimpleCluster()
{
}

double SimpleCluster::getVariance()
{
    double numObs = (double)this->numObs;
    double avgX = this->linearSumX / numObs;
    double avgY = this->linearSumY / numObs;
    return (this->squareSum + numObs * ((avgX * avgX) + (avgY * avgY)) - 2 * ((avgX * this->linearSumX) + (avgY * this->linearSumY))) / numObs;
}

bool SimpleCluster::checkAndMerge(shared_ptr<SimpleCluster> cluster, double varianceThreshold)
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
        // two cluster which break the VARIANCE_THRESHOLD -> the whole clustering is finished
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
        return true;
    }
}

double SimpleCluster::distanceTo(shared_ptr<SimpleCluster> cluster)
{
    return sqrt(pow(this->x - cluster->x, 2) + pow(this->y - cluster->y, 2));
}

} /* namespace msl */
