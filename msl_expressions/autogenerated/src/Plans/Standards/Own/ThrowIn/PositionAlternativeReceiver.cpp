using namespace std;
#include "Plans/Standards/Own/ThrowIn/PositionAlternativeReceiver.h"

/*PROTECTED REGION ID(inccpp1462978634990) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "SystemConfig.h"
#include "engine/model/EntryPoint.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
#include <RawSensorData.h>
#include <MSLWorldModel.h>
#include <Ball.h>
using geometry::CNPointAllo;
using geometry::CNPointEgo;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1462978634990) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    PositionAlternativeReceiver::PositionAlternativeReceiver() :
            DomainBehaviour("PositionAlternativeReceiver")
    {
        /*PROTECTED REGION ID(con1462978634990) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    PositionAlternativeReceiver::~PositionAlternativeReceiver()
    {
        /*PROTECTED REGION ID(dcon1462978634990) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void PositionAlternativeReceiver::run(void* msg)
    {
        /*PROTECTED REGION ID(run1462978634990) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto egoBallPos = wm->ball->getPositionEgo();
        if (!ownPos || !egoBallPos)
        {
            return;
        }
        auto alloBall = egoBallPos->toAllo(*ownPos);
        // Create additional points for path planning
        auto additionalPoints = nonstd::make_optional<vector<geometry::CNPointAllo>>();
        // add alloBall to path planning
        additionalPoints->push_back(alloBall);

        msl_actuator_msgs::MotionControl mc;
        CNPointAllo alloTarget;
        CNPointEgo egoTarget;

        if (alloBall.y < 0)
        {
            alloTarget.y = alloBall.y + 2300.0;
        }
        else
        {
            alloTarget.y = alloBall.y - 2300.0;
        }

        alloTarget.x = alloBall.x;

        egoTarget = alloTarget.toEgo(*ownPos);

//        mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoBallPos, 0, additionalPoints);
        query.egoDestinationPoint = egoTarget;
        query.egoAlignPoint = egoBallPos;
        query.additionalPoints = additionalPoints;
        mc = rm.moveToPoint(query);

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
    void PositionAlternativeReceiver::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1462978634990) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1462978634990) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
