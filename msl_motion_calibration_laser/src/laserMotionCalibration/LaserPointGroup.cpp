/*
 * LaserPointGroup.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: cn
 */

#include "laserMotionCalibration/LaserPointGroup.h"
#include "laserMotionCalibration/MotionCalibrationLaser.h"
#include <math.h>

namespace laserMotionCalibration
{
supplementary::SystemConfig *LaserPointGroup::sc = supplementary::SystemConfig::getInstance();
double LaserPointGroup::pillarRadius = (*sc)["LaserLocalization"]->get<double>("LaserLocalization.pillarRadius", NULL);
double LaserPointGroup::hokuyoCenterOffset = (*sc)["LaserLocalization"]->get<double>("LaserLocalization.hokuyoCenterOffset", NULL);

LaserPointGroup::LaserPointGroup()
{
}

LaserPointGroup::~LaserPointGroup()
{
}

double LaserPointGroup::getCenter()
{
    return (points.at(points.size() - 1).index + points.at(0).index) / 2.0;
}

double LaserPointGroup::getDistance()
{
    return 1000.0 * points.at(points.size() / 2).distance;
}

std::shared_ptr<geometry::CNPoint2D> LaserPointGroup::getWeightedCenter()
{
    double x = 0, y = 0, intensities = 0;
    for (auto point : points)
    {
        x += point.getXY()->x * point.intensity;
        y += point.getXY()->y * point.intensity;
        intensities += point.intensity;
    }
    std::shared_ptr<geometry::CNPoint2D> weightedCenter = std::make_shared<geometry::CNPoint2D>();
    weightedCenter->x = x / intensities;
    weightedCenter->y = y / intensities;
    return weightedCenter;
}

std::shared_ptr<geometry::CNPoint2D> LaserPointGroup::getPillarCenter()
{
    // convert hokuyoCenterOffset and pillarRadius from mm to m because weighted center's coordinates are also in m
    std::shared_ptr<geometry::CNPoint2D> pillarCenter = getWeightedCenter();
    double length = pillarCenter->length();
    double growth = sqrt((length + (pillarRadius + hokuyoCenterOffset) / 1000.0) / length);
    pillarCenter->x *= growth;
    pillarCenter->y *= growth;
    return pillarCenter;
}

} /* namespace laserMotionCalibration */
