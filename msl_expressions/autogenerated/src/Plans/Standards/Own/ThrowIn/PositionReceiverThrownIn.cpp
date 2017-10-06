using namespace std;
#include "Plans/Standards/Own/ThrowIn/PositionReceiverThrownIn.h"

/*PROTECTED REGION ID(inccpp1461584204507) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "SystemConfig.h"
#include "engine/model/EntryPoint.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
#include "engine/constraintmodul/Query.h"
#include "GSolver.h"
#include "SolverType.h"
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <Ball.h>
using geometry::CNPointAllo;
using geometry::CNPointEgo;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1461584204507) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    PositionReceiverThrownIn::PositionReceiverThrownIn() :
            DomainBehaviour("PositionReceiverThrownIn")
    {
        /*PROTECTED REGION ID(con1461584204507) ENABLED START*/ //Add additional options here
        this->query = make_shared < alica::Query > (this->wm->getEngine());
        /*PROTECTED REGION END*/
    }
    PositionReceiverThrownIn::~PositionReceiverThrownIn()
    {
        /*PROTECTED REGION ID(dcon1461584204507) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void PositionReceiverThrownIn::run(void* msg)
    {
        /*PROTECTED REGION ID(run1461584204507) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto egoBallPos = wm->ball->getPositionEgo();
        if (!ownPos || !egoBallPos)
        {
            return;
        }
        auto alloBall = egoBallPos->toAllo(*ownPos);
        // Create additional points for path planning
        auto additionalPoints = nonstd::make_optional<
                vector<CNPointAllo>>();
        // add alloBall to path planning
        additionalPoints->push_back(alloBall);
        alloTarget.y = alloBall.y;
        alloTarget.x = alloBall.x - 2300;
        auto egoTarget = alloTarget.toEgo(*ownPos);

        msl_actuator_msgs::MotionControl mc;

        // ask the path planner how to get there
        // replaced with new moveToPoint method
//        mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoBallPos, 0, additionalPoints);
        mQuery.egoDestinationPoint = egoTarget;
        mQuery.egoAlignPoint = egoBallPos;
        mQuery.additionalPoints = additionalPoints;
        mc = rm.moveToPoint(mQuery);

        // if we reach the point and are aligned, the behavior is successful
        if (egoTarget.length() < 250 && fabs(egoBallPos->rotateZ(M_PI).angleZ()) < (M_PI / 180) * 5)
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
    void PositionReceiverThrownIn::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1461584204507) ENABLED START*/ //Add additional options here
        query->clearDomainVariables();
        query->addDomainVariable(wm->getOwnId(), "x");
        query->addDomainVariable(wm->getOwnId(), "y");
        result.clear();
        string tmp;
        bool success = true;
        alloTarget = CNPointAllo(0, 0);
        try
        {
            success &= getParameter("TeamMateTaskName", tmp);
            if (success)
            {
                taskName = tmp;
            }
        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "PRT: Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1461584204507) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
