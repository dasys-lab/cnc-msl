using namespace std;
#include "Plans/Behaviours/PositionReceiver.h"

/*PROTECTED REGION ID(inccpp1439379316897) ENABLED START*/ //Add additional includes here
using geometry::CNPointEgo;
using geometry::CNPointAllo;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1439379316897) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    PositionReceiver::PositionReceiver() :
            DomainBehaviour("PositionReceiver")
    {
        /*PROTECTED REGION ID(con1439379316897) ENABLED START*/ //Add additional options here
        readConfigParameters();
        /*PROTECTED REGION END*/
    }
    PositionReceiver::~PositionReceiver()
    {
        /*PROTECTED REGION ID(dcon1439379316897) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void PositionReceiver::run(void* msg)
    {
        /*PROTECTED REGION ID(run1439379316897) ENABLED START*/ //Add additional options here
        //TODO  not allowed in enemy half (rules), new conf for rules
        msl::RobotMovement rm;
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto egoBallPos = wm->ball->getPositionEgo();

        if (!ownPos || !egoBallPos)
        {
            return;
        }

        auto alloBall = egoBallPos->toAllo(*ownPos);

        // Create additional points for path planning
        nonstd::optional<std::vector<geometry::CNPointAllo>> additionalPoints = nonstd::make_optional(
                vector<geometry::CNPointAllo>());
        // add alloBall to path planning
        additionalPoints->push_back(alloBall);

        //set target point as (0,-2300)
        CNPointEgo egoTarget = CNPointAllo(0, -ballDistanceRec).toEgo(*ownPos);

        msl_actuator_msgs::MotionControl mc;

        msl::MSLWorldModel* wm = msl::MSLWorldModel::get();
        if (wm->game->getSituation() == msl::Situation::Start)
        { // they already pressed start and we are still positioning, so speed up!
          // remeoved with new moveToPoint method
//            mc = msl::RobotMovement::moveToPointFast(egoTarget, egoBallPos, fastCatchRadius, additionalPoints);
            query->egoDestinationPoint = egoTarget;
            query->egoAlignPoint = egoBallPos;
            query->snapDistance = fastCatchRadius;
            query->additionalPoints = additionalPoints;
            query->velocityMode = msl::MovementQuery::Velocity::FAST;
            mc = rm.moveToPoint(query);
        }
        else
        { // still enough time to position ...
//            mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoBallPos, slowCatchRadius, additionalPoints);
            query->egoDestinationPoint = egoTarget;
            query->egoAlignPoint = egoBallPos;
            query->snapDistance = slowCatchRadius;
            query->additionalPoints = additionalPoints;
            query->velocityMode = msl::MovementQuery::Velocity::DEFAULT;
            mc = rm.moveToPoint(query);
        }

        // if we reach the point and are aligned, the behavior is successful
        if (mc.motion.translation == 0 && fabs(egoBallPos->rotateZ(M_PI).angleZ()) < (M_PI / 180) * alignTolerance)
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
        /*PROTECTED REGION ID(initialiseParameters1439379316897) ENABLED START*/ //Add additional options here
        readConfigParameters();
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1439379316897) ENABLED START*/ //Add additional methods here
    void PositionReceiver::readConfigParameters()
    {
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        fastCatchRadius = (*sc)["Drive"]->get<double>("Drive.Fast.CatchRadius", NULL);
        slowCatchRadius = (*sc)["Drive"]->get<double>("Drive.Carefully.CatchRadius", NULL);
        alignTolerance = (*sc)["Drive"]->get<double>("Drive.Default.AlignTolerance", NULL);
        ballDistanceRec = (*sc)["Drive"]->get<double>("Drive.KickOff.BallDistRec", NULL);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
