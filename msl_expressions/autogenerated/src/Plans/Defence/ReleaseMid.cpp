using namespace std;
#include "Plans/Defence/ReleaseMid.h"

/*PROTECTED REGION ID(inccpp1458033482289) ENABLED START*/ // Add additional includes here
#include "engine/RunningPlan.h"
#include "engine/model/AbstractPlan.h"
#include "msl_robot/robotmovement/RobotMovement.h"
#include <Ball.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <Robots.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1458033482289) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
ReleaseMid::ReleaseMid()
    : DomainBehaviour("ReleaseMid")
{
    /*PROTECTED REGION ID(con1458033482289) ENABLED START*/ // Add additional options here
    teamMateTaskName = "";
    teamMatePlanName = "";
    ep = nullptr;
    teamMateId = 0;
    threshold = 0.0;
    yHysteresis = 0.0;
    vMax = 0.0;
    /*PROTECTED REGION END*/
}
ReleaseMid::~ReleaseMid()
{
    /*PROTECTED REGION ID(dcon1458033482289) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void ReleaseMid::run(void *msg)
{
    /*PROTECTED REGION ID(run1458033482289) ENABLED START*/ // Add additional options here
    msl::RobotMovement rm;


    msl_actuator_msgs::MotionControl mc;
    auto egoBallPos = wm->ball->getPositionEgo();
    auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
    if (ownPos)
    {
        mc = rm.driveRandomly(500);
        send(mc);
        cout << "AAPR: OwnPos is null" << endl;
        return;
    }

    if (ep == nullptr)
    {
        cout << this->getRunningPlan()->getPlan()->getName() << ": EP is null" << endl;
        return;
    }

    // the only teammate in the corresponding task/ entrypoint
    auto teammates = robotsInEntryPointOfHigherPlan(ep);
    if (teammates)
    {
        for (int mateId : *teammates)
        {
            this->teamMateId = mateId;
            break;
        }
    }

    // determine the best reference point
    geometry::CNPointAllo referencePoint; // Point we want to align and pos to
    if (this->teamMateId != 0)
    { // take the teammate as reference point
        auto teammate = wm->robots->teammates.getTeammatePositionBuffer(teamMateId).getLastValidContent();
        referencePoint = geometry::CNPointAllo(teammate->x, teammate->y);
    }
    else if (egoBallPos)
    { // take the ball as reference point
        referencePoint = egoBallPos->toAllo(*ownPos);
    }
    else
    { // no teammate and no ball, hmpf stay inside the middle of the field
        referencePoint = geometry::CNPointAllo(4000.0, 0.0);
    }

    // point we want to drive to
    // because of 0.0, this behaviour should be triggered
    geometry::CNPointAllo targetPoint(min(max(0.0, referencePoint.x - 2000.0), 3000.0), (referencePoint.y * 2) / 3);
    if (abs(targetPoint.y - referencePoint.y) < 500.0)
    {
        if (referencePoint.y > threshold)
        {
            threshold = -yHysteresis;
            targetPoint.y = referencePoint.y - 500.0;
        }
        else
        {
            threshold = +yHysteresis;
            targetPoint.y = referencePoint.y + 500.0;
        }
    }
    // repaced moveToPointCarefully with new moveToPoint method
    query.egoDestinationPoint = targetPoint.toEgo(*ownPos);
    query.snapDistance = 50;
    if (egoBallPos)
    {
        query.egoAlignPoint = egoBallPos;
        mc = rm.moveToPoint(query);
    }
    else
    {
        query.egoAlignPoint = referencePoint.toEgo(*ownPos);
        mc = rm.moveToPoint(query);
    }
    if (!std::isnan(mc.motion.translation))
    {
        send(mc);
    }
    /*PROTECTED REGION END*/
}
void ReleaseMid::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1458033482289) ENABLED START*/ // Add additional options here
    supplementary::SystemConfig *sc = supplementary::SystemConfig::getInstance();
    vMax = (*this->sc)["Behaviour"]->get<double>("Behaviour", "MaxSpeed", NULL);
    bool success = true;
    try
    {
        string tmp = "";
        success &= getParameter("YHysteresis", tmp);
        if (success)
        {
            this->yHysteresis = stod(tmp);
        }
        success &= getParameter("TeamMatePlanName", teamMatePlanName);
        success &= getParameter("TeamMateTaskName", teamMateTaskName);
    }
    catch (exception &e)
    {
        cerr << "Could not cast the parameter properly" << endl;
    }
    if (!success)
    {
        cerr << "StandardAlignAndGrab: Parameter does not exist" << endl;
    }

    ep = getHigherEntryPoint(teamMatePlanName, teamMateTaskName);
    if (ep == nullptr)
    {
        cerr << "ReleaseMid: Receiver==null, because planName, teamMateTaskName does not match" << endl;
    }
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1458033482289) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
