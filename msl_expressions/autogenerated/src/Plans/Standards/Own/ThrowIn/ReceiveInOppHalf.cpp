using namespace std;
#include "Plans/Standards/Own/ThrowIn/ReceiveInOppHalf.h"

/*PROTECTED REGION ID(inccpp1462370340143) ENABLED START*/ // Add additional includes here
#include <Ball.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <RawSensorData.h>
#include <Robots.h>
#include <SystemConfig.h>
#include <engine/Assignment.h>
#include <engine/RunningPlan.h>
#include <engine/model/EntryPoint.h>
#include <engine/model/Plan.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/robotmovement/RobotMovement.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1462370340143) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
ReceiveInOppHalf::ReceiveInOppHalf()
    : DomainBehaviour("ReceiveInOppHalf")
{
    /*PROTECTED REGION ID(con1462370340143) ENABLED START*/ // Add additional options here
    this->mQuery = make_shared<msl::MovementQuery>();
    /*PROTECTED REGION END*/
}
ReceiveInOppHalf::~ReceiveInOppHalf()
{
    /*PROTECTED REGION ID(dcon1462370340143) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void ReceiveInOppHalf::run(void *msg)
{
    /*PROTECTED REGION ID(run1462370340143) ENABLED START*/ // Add additional options here
    auto ownPos = wm->rawSensorData->getOwnPositionVision();
    auto alloBallPos = wm->ball->getAlloBallPosition();
    if (!ownPos || !alloBallPos)
        return;

    // Create additional points for path planning
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    // add alloBall to path planning
    additionalPoints->push_back(alloBallPos);

    double yCordOfReceiver = 0.0;
    double securityReceiver = 40.0;
    if (alloBallPos->y < 0.0) // right side line
    {
        // place the receiver 1m outside the sideline
        //            if (wm->field->getSurrounding() > 1300)
        //            {
        //                yCordOfReceiver = -wm->field->getFieldWidth() / 2.0 - 1000.0;
        //            }
        //            else
        //            {
        //                yCordOfReceiver = -wm->field->getFieldWidth() / 2.0 - wm->field->getSurrounding()
        //                        + wm->pathPlanner->getRobotRadius() + securityReceiver;
        //            }

        // ^ forbidden with rule changes 2017. place robot on side line instead
        yCordOfReceiver = -wm->field->getFieldWidth() / 2.0 + securityReceiver;
    }
    else // left side line
    {
        // place the receiver 1m outside the sideline
        //			if (wm->field->getSurrounding() > 1300)
        //			{
        //				yCordOfReceiver = wm->field->getFieldWidth() / 2.0 + 1000.0;
        //			}
        //			else
        //			{
        //				yCordOfReceiver = wm->field->getFieldWidth() / 2.0 + wm->field->getSurrounding()
        //						- wm->pathPlanner->getRobotRadius() - securityReceiver;
        //			}

        // ^ forbidden with rule changes 2017. place robot on side line instead

        yCordOfReceiver = wm->field->getFieldWidth() / 2.0 - securityReceiver;
    }

    double lowestX = wm->field->getFieldLength() / 2;
    auto opps = wm->robots->opponents.getOpponentsAlloClustered();
    for (auto opp : *opps)
    {
        double distToLine =
            geometry::distancePointToLineSegment(opp->x, opp->y, 2000, yCordOfReceiver, wm->field->getFieldLength() / 2 - 2000, yCordOfReceiver);
        if (distToLine > 3000)
        {
            continue;
        }

        if (lowestX > opp->x)
        {
            lowestX = opp->x;
        }
    }

    alloTarget = make_shared<geometry::CNPoint2D>(wm->field->getFieldLength() / 4, yCordOfReceiver);

    if (lowestX < wm->field->getFieldLength() / 2 - 2000)
    { // opponent close to pass line
        alloTarget->x = min(alloTarget->x, max(lowestX - 2000, 2000.0));
    }

    mQuery->egoDestinationPoint = alloTarget->alloToEgo(*ownPos);
    mQuery->egoAlignPoint = alloBallPos->alloToEgo(*ownPos);
    mQuery->snapDistance = this->snapDist;
    mQuery->additionalPoints = additionalPoints;
    msl_actuator_msgs::MotionControl mc = this->robot->robotMovement->moveToPoint(mQuery);

    if (!std::isnan(mc.motion.translation))
    {
        send(mc);
    }
    /*PROTECTED REGION END*/
}
void ReceiveInOppHalf::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1462370340143) ENABLED START*/ // Add additional options here
    this->securityReceiver = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "securityReceiver", NULL);
    this->snapDist = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "snapDist", NULL);
    this->oppFarAwayDist = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "oppFarAwayDist", NULL);
    string tmp;
    bool success = true;
    alloTarget = make_shared<geometry::CNPoint2D>(0, 0);
    try
    {
        success &= getParameter("TeamMateTaskName", tmp);
        if (success)
        {
            taskName = tmp;
        }
    }
    catch (exception &e)
    {
        cerr << "Could not cast the parameter properly" << endl;
    }
    if (!success)
    {
        cerr << "PRT: Parameter does not exist" << endl;
    }
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1462370340143) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
