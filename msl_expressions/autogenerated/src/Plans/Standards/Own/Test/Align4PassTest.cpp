#include "Plans/Standards/Own/Test/Align4PassTest.h"

/*PROTECTED REGION ID(inccpp1513609382468) ENABLED START*/ // Add additional includes here
#include <SystemConfig.h>
#include <MSLWorldModel.h>
#include <Ball.h>
#include <RawSensorData.h>
#include <msl_robot/MSLRobot.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1513609382468) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
Align4PassTest::Align4PassTest()
    : DomainBehaviour("Align4PassTest")
{
    /*PROTECTED REGION ID(con1513609382468) ENABLED START*/ // Add additional options here
    this->recBallDist = (*this->sc)["StandardSituation"]->get<double>("PassTest", "ReceiverDistance", NULL);
    this->m_Query = make_shared<msl::MovementQuery>();
    /*PROTECTED REGION END*/
}
Align4PassTest::~Align4PassTest()
{
    /*PROTECTED REGION ID(dcon1513609382468) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void Align4PassTest::run(void *msg)
{
    /*PROTECTED REGION ID(run1513609382468) ENABLED START*/ // Add additional options here

    shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision();
    shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball->getEgoBallPosition();

    // return if necessary information is missing
    if (ownPos == nullptr || egoBallPos == nullptr)
    {
        return;
    }

    // Create allo ball
    shared_ptr<geometry::CNPoint2D> alloBall = egoBallPos->egoToAllo(*ownPos);

    // Create additional points for path planning
    auto additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    // add alloBall to path planning
    additionalPoints->push_back(alloBall);
    shared_ptr<geometry::CNPoint2D> egoTarget;
    MotionControl mc;
    auto rob = msl::MSLRobot::get();

    if (!oldBallPos)
    {
        oldBallPos = alloBall;
    }
    if (!alloTarget || oldBallPos->distanceTo(alloBall) > 500)
    { // recalculate alloReceiverTarget if the ball moved more than "receiverBallMovedThreshold" mm

        oldBallPos = alloBall;

        // calculate a point that is "receiverDistanceToBall" away from ball towards field mid (0,0).
        alloTarget = (alloBall + (alloBall->normalize() * -this->recBallDist));
    }

    // ask the path planner how to get there
    egoTarget = alloTarget->alloToEgo(*ownPos);
    this->m_Query->egoDestinationPoint = egoTarget;
    this->m_Query->egoAlignPoint = egoBallPos;
    this->m_Query->additionalPoints = additionalPoints;
    mc = rob->robotMovement->moveToPoint(m_Query);

    // if we reach the point and are aligned, the behavior is successful
    if (egoTarget->length() < 100 && fabs(egoBallPos->rotate(M_PI)->angleTo()) < 5)
    {
        this->setSuccess(true);
    }

    if (!std::isnan(mc.motion.translation))
    {
        send(mc);
    }
    /*PROTECTED REGION END*/
}
void Align4PassTest::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1513609382468) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1513609382468) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
