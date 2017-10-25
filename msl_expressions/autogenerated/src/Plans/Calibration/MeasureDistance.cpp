using namespace std;
#include "Plans/Calibration/MeasureDistance.h"

/*PROTECTED REGION ID(inccpp1508940928986) ENABLED START*/ // Add additional includes here
#include <LaserScannerPosition.h>
#include <MSLWorldModel.h>
#include <msl_actuator_msgs/RawOdometryInfo.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1508940928986) ENABLED START*/ // initialise static variables here
double MeasureDistance::distanceLaser;
double MeasureDistance::distanceOdometry;
shared_ptr<geometry::CNPoint2D> MeasureDistance::startPositionOdometry = make_shared<geometry::CNPoint2D>();
shared_ptr<geometry::CNPoint2D> MeasureDistance::startPositionLaser = make_shared<geometry::CNPoint2D>();
/*PROTECTED REGION END*/
MeasureDistance::MeasureDistance()
    : DomainBehaviour("MeasureDistance")
{
    /*PROTECTED REGION ID(con1508940928986) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
MeasureDistance::~MeasureDistance()
{
    /*PROTECTED REGION ID(dcon1508940928986) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void MeasureDistance::run(void *msg)
{
    /*PROTECTED REGION ID(run1508940928986) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void MeasureDistance::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1508940928986) ENABLED START*/ // Add additional options here
	MeasureDistance::startPositionOdometry->x = wm->rawOdometry->position.x;
    MeasureDistance::startPositionOdometry->y = wm->rawOdometry->position.y;
    shared_ptr<geometry::CNPoint2D> laserPosition = wm->laserScannerPosition->getPosition();
    MeasureDistance::startPositionLaser->x = laserPosition->x;
    MeasureDistance::startPositionLaser->y = laserPosition->y;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1508940928986) ENABLED START*/ // Add additional methods here
double MeasureDistance::getOdometryDistance(DomainBehaviour* beh)
{
    shared_ptr<geometry::CNPoint2D> currentPositionOdometry = make_shared<geometry::CNPoint2D>(beh->wm->rawOdometry->position.x, beh->wm->rawOdometry->position.y);
    MeasureDistance::distanceOdometry = (currentPositionOdometry - startPositionOdometry)->length();
    return MeasureDistance::distanceOdometry;
}
double MeasureDistance::getLaserDistance(DomainBehaviour* beh)
{
    shared_ptr<geometry::CNPoint2D> currentPositionLaser = beh->wm->laserScannerPosition->getPosition();
    MeasureDistance::distanceLaser = (currentPositionLaser - startPositionLaser)->length();
    return distanceLaser;
}
/*PROTECTED REGION END*/
} /* namespace alica */
