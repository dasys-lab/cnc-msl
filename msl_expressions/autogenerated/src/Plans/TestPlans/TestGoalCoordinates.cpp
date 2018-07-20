using namespace std;
#include "Plans/TestPlans/TestGoalCoordinates.h"

/*PROTECTED REGION ID(inccpp1532092850344) ENABLED START*/ // Add additional includes here
#include <LaserScanner.h>
#include <MSLFootballField.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <msl_sensor_msgs/LaserLocalization.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1532092850344) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
TestGoalCoordinates::TestGoalCoordinates()
    : DomainBehaviour("TestGoalCoordinates")
{
    /*PROTECTED REGION ID(con1532092850344) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
TestGoalCoordinates::~TestGoalCoordinates()
{
    /*PROTECTED REGION ID(dcon1532092850344) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void TestGoalCoordinates::run(void *msg)
{
    /*PROTECTED REGION ID(run1532092850344) ENABLED START*/ // Add additional options here
    auto ownPosition = wm->rawSensorData->getOwnPositionVision();
    if (ownPosition == nullptr)
    {
        return;
    }
    auto position = wm->laserScanner->getGoalWallPosition();
    if (position == nullptr)
    {
        return;
    }
    geometry::CNPoint2D egoLeftGoalPos = geometry::CNPoint2D(position->points[0].x, position->points[0].y);
    geometry::CNPoint2D egoRightGoalPos = geometry::CNPoint2D(position->points[1].x, position->points[1].y);

    auto alloLeftGoalPos = egoLeftGoalPos.egoToAllo(*ownPosition);
    auto alloRighGoalPos = egoRightGoalPos.egoToAllo(*ownPosition);

    auto configAlloLeftGoalPos = wm->field->posLeftOppGoalPost();
    auto configAlloRightGoalPos = wm->field->posRightOppGoalPost();

    cout << "Left Goal post: laser scan [" << alloLeftGoalPos->x << "|" << alloLeftGoalPos->y << "]" << endl;
    cout << "Left Goal post: laser scan [" << configAlloLeftGoalPos->x << "|" << configAlloLeftGoalPos->y << "]" << endl;
    cout << "Right Goal post: config [" << alloRighGoalPos->x << "|" << alloRighGoalPos->y << "]" << endl;
    cout << "Right Goal post: config [" << configAlloRightGoalPos->x << "|" << configAlloRightGoalPos->y << "]" << endl;

    cout << "Diff left Goal post: [" << fabs(configAlloRightGoalPos->x - alloLeftGoalPos->x) << "|"
    		<< fabs(configAlloRightGoalPos->y - alloLeftGoalPos->y) << "]" << endl;
    cout << "Diff right Goal post: [" << fabs(configAlloRightGoalPos->x - alloRighGoalPos->x) << "|"
    		<< fabs(configAlloRightGoalPos->y - alloRighGoalPos->y) << "]" << endl;
    /*PROTECTED REGION END*/
}
void TestGoalCoordinates::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1532092850344) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1532092850344) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
