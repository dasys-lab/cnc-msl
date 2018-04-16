using namespace std;
#include "Plans/Standards/Own/ThrowIn/ReceiveInOppHalf.h"

/*PROTECTED REGION ID(inccpp1462370340143) ENABLED START*/ // Add additional includes here
#include <Ball.h>
#include <MSLFootballField.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <Robots.h>
#include <engine/Assignment.h>
#include <engine/RunningPlan.h>
#include <engine/model/EntryPoint.h>
#include <engine/model/Plan.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/robotmovement/MovementQuery.h>
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
    this->yCoordOfReceiver = 0.0;
    this->maxIterations = 0;
    this->snapDist = 0.0;
    this->posSignCounter = 0;
    this->itCounter = 0;
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
    auto ownPos = this->wm->rawSensorData->getOwnPositionVision();
    auto alloBallPos = this->wm->ball->getAlloBallPosition();
    if (!ownPos || !alloBallPos)
    {
        return;
    }

    // Create additional points for path planning
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    // add alloBall to path planning
    additionalPoints->push_back(alloBallPos);

    if (this->itCounter < this->maxIterations)
    {
        if (alloBallPos->y >= 0.0)
        {
            ++this->posSignCounter;
        }
        ++this->itCounter;
        return;
    }

    if (this->posSignCounter >= this->maxIterations / 2) // right side line
    {
        this->yCoordOfReceiver = this->wm->field->getFieldWidth() / 2.0 - this->securityReceiver;
    }
    else // left side line
    {
        this->yCoordOfReceiver = -this->wm->field->getFieldWidth() / 2.0 + this->securityReceiver;
    }

    double lowestX = this->wm->field->getFieldLength() / 2;
    auto opps = this->wm->robots->opponents.getOpponentsAlloClustered();
    for (auto opp : *opps)
    {
        double distToLine = geometry::distancePointToLineSegment(opp->x, opp->y, 2000, this->yCoordOfReceiver, this->wm->field->getFieldLength() / 2 - 2000,
                                                                 this->yCoordOfReceiver);
        if (distToLine > 3000)
        {
            continue;
        }

        if (lowestX > opp->x)
        {
            lowestX = opp->x;
        }
    }

    this->alloTarget = make_shared<geometry::CNPoint2D>(this->wm->field->getFieldLength() / 4, this->yCoordOfReceiver);

    if (lowestX < this->wm->field->getFieldLength() / 2 - 2000)
    { // opponent close to pass line
        this->alloTarget->x = min(alloTarget->x, max(lowestX - 2000, 2000.0));
    }

    this->mQuery->egoDestinationPoint = this->alloTarget->alloToEgo(*ownPos);
    this->mQuery->egoAlignPoint = alloBallPos->alloToEgo(*ownPos);
    this->mQuery->snapDistance = this->snapDist;
    this->mQuery->additionalPoints = additionalPoints;
    msl_actuator_msgs::MotionControl mc = this->robot->robotMovement->moveToPoint(this->mQuery);

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
    this->maxIterations = (*this->sc)["Behaviour"]->get<double>("ReceiveInOppHalf", "maxIterations", NULL);
    this->alloTarget = make_shared<geometry::CNPoint2D>(0, 0);
    this->yCoordOfReceiver = 0.0;
    this->posSignCounter = 0;
    this->itCounter = 0;
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1462370340143) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
