using namespace std;
#include "Plans/Attack/PassIntoFreeZone.h"

/*PROTECTED REGION ID(inccpp1508951632953) ENABLED START*/
#include "GSolver.h"
#include "SolverType.h"
#include "engine/constraintmodul/Query.h"
#include "msl_robot/robotmovement/RobotMovement.h"
#include "msl_helper_msgs/DebugMsg.h"
#include <Ball.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
using nonstd::make_optional;
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1508951632953) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
PassIntoFreeZone::PassIntoFreeZone()
    : DomainBehaviour("PassIntoFreeZone")
{
    /*PROTECTED REGION ID(con1508951632953) ENABLED START*/ // Add additional options here
    this->query = make_shared<alica::Query>(this->wm->getEngine());
    /*PROTECTED REGION END*/
}
PassIntoFreeZone::~PassIntoFreeZone()
{
    /*PROTECTED REGION ID(dcon1508951632953) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void PassIntoFreeZone::run(void *msg)
{
    /*PROTECTED REGION ID(run1508951632953) ENABLED START*/
    // Add additional options here
    /*
     * TODO:
     * 1. implement constraint for defining pass point
     * 2. implement dribbling for passing to pass point (don't forget to send pass message)
     * 3. play the pass (chose kickpower wisely)
     */
    auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
    auto ballPos = wm->ball->getPositionEgo();

    if (!ownPos || !ballPos)
    {
        return;
    }
    auto alloBall = ballPos->toAllo(*ownPos);

    msl_actuator_msgs::MotionControl mc;
    if (query->getSolution(SolverType::GRADIENTSOLVER, runningPlan, result) || result.size() > 1)
    {
        cout << this->getName() << ": FOUND a solution!" << endl;
        auto additionalPoints = make_optional<vector<geometry::CNPointAllo>>();
        additionalPoints->push_back(alloBall);
        geometry::CNPointAllo alloTarget(result.at(0), result.at(1));

        cout << this->getName() << ": Target x,y: " << alloTarget.x << " " << alloTarget.y << endl;
        msl_helper_msgs::DebugMsg dm;
        msl_helper_msgs::DebugPoint dp;
        dp.red = 255;
        dp.green = 0;
        dp.blue = 0;
        dp.radius = 100;
        dp.point.x = alloTarget.x;
        dp.point.y = alloTarget.y;
        dm.points.push_back(dp);
        this->send(dm);

        msl::RobotMovement rm;
        mQuery.egoDestinationPoint = make_optional<geometry::CNPointEgo>(alloTarget.toEgo(*ownPos));
        mQuery.egoAlignPoint = make_optional<geometry::CNPointEgo>(alloBall.toEgo(*ownPos));
        mQuery.snapDistance = 100;
        mQuery.additionalPoints = additionalPoints;
        mc = rm.moveToPoint(mQuery);
    }
    else
    {
        cout << this->getName() << ": Did not get a filled result vector!" << endl;
    }
    if (!std::isnan(mc.motion.translation))
    {
        send(mc);
    }
    else
    {
        cout << "Motion command is NaN!" << endl;
    }
    /*PROTECTED REGION END*/
}
void PassIntoFreeZone::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1508951632953) ENABLED START*/ // Add additional options here
    query->clearDomainVariables();
    query->addDomainVariable(wm->getOwnId(), "x");
    query->addDomainVariable(wm->getOwnId(), "y");
    result.clear();
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1508951632953) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
