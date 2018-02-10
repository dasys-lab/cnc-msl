using namespace std;
#include "Plans/Behaviours/PositionReceiver.h"

/*PROTECTED REGION ID(inccpp1439379316897) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <MSLWorldModel.h>
#include <pathplanner/PathProxy.h>
#include <pathplanner/evaluator/PathEvaluator.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <Game.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1439379316897) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
PositionReceiver::PositionReceiver()
    : DomainBehaviour("PositionReceiver")
{
    /*PROTECTED REGION ID(con1439379316897) ENABLED START*/ // Add additional options here
    query = make_shared<msl::MovementQuery>();
    readConfigParameters();
    /*PROTECTED REGION END*/
}
PositionReceiver::~PositionReceiver()
{
    /*PROTECTED REGION ID(dcon1439379316897) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void PositionReceiver::run(void *msg)
{
    /*PROTECTED REGION ID(run1439379316897) ENABLED START*/ // Add additional options here
    // TODO  not allowed in enemy half (rules), new conf for rules
    shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision();
    auto egoBallPos = wm->ball->getEgoBallPosition();

    if (ownPos == nullptr || egoBallPos == nullptr)
    {
        return;
    }

    shared_ptr<geometry::CNPoint2D> alloBall = egoBallPos->egoToAllo(*ownPos);

    // Create additional points for path planning
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
    // add alloBall to path planning
    additionalPoints->push_back(alloBall);

    // set target point as (0,-2300)
    shared_ptr<geometry::CNPoint2D> egoTarget = make_shared<geometry::CNPoint2D>(0, -ballDistanceRec)->alloToEgo(*ownPos);

    msl_actuator_msgs::MotionControl mc;

    msl::MSLWorldModel *wm = msl::MSLWorldModel::get();

    if (wm->game->getSituation() == msl::Situation::Start)
    {   // they already pressed start and we are still positioning, so speed up!
        // removed with new moveToPoint method
        query->snapDistance = fastCatchRadius;
        query->velocityMode = msl::VelocityMode::FAST;
    }
    else
    { // still enough time to position ...
        query->snapDistance = slowCatchRadius;
        query->velocityMode = msl::VelocityMode::DEFAULT;
    }

    query->egoDestinationPoint = egoTarget;
    query->egoAlignPoint = egoBallPos;
    query->additionalPoints = additionalPoints;
    mc = this->robot->robotMovement->moveToPoint(query);

    // if we reach the point and are aligned, the behavior is successful
    if (mc.motion.translation == 0 && fabs(egoBallPos->rotate(M_PI)->angleTo()) < (M_PI / 180) * alignTolerance)
    {
        this->setSuccess(true);
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
void PositionReceiver::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1439379316897) ENABLED START*/ // Add additional options here
    supplementary::SystemConfig *sc = supplementary::SystemConfig::getInstance();
    fastCatchRadius = (*sc)["Drive"]->get<double>("Drive.Fast.CatchRadius", NULL);
    slowCatchRadius = (*sc)["Drive"]->get<double>("Drive.Carefully.CatchRadius", NULL);
    alignTolerance = (*sc)["Drive"]->get<double>("Drive.Default.AlignTolerance", NULL);
    ballDistanceRec = (*sc)["Drive"]->get<double>("Drive.KickOff.BallDistRec", NULL);
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1439379316897) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
