using namespace std;
#include "Plans/Standards/Own/PassIntoPath/StandardAlignToPassPos.h"

/*PROTECTED REGION ID(inccpp1457532279657) ENABLED START*/ // Add additional includes here
#include "SolverType.h"
#include "msl_robot/robotmovement/RobotMovement.h"
#include <Ball.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <msl_helper_msgs/PassMsg.h>
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1457532279657) ENABLED START*/ // initialise static variables here
/*PROTECTED REGION END*/
StandardAlignToPassPos::StandardAlignToPassPos()
    : DomainBehaviour("StandardAlignToPassPos")
{
    /*PROTECTED REGION ID(con1457532279657) ENABLED START*/ // Add additional options here
    query = make_shared<Query>(wm->getEngine());

    supplementary::SystemConfig *sys = supplementary::SystemConfig::getInstance();
    maxVel = (*sys)["Behaviour"]->get<double>("Behaviour.MaxSpeed", NULL);
    /*PROTECTED REGION END*/
}
StandardAlignToPassPos::~StandardAlignToPassPos()
{
    /*PROTECTED REGION ID(dcon1457532279657) ENABLED START*/ // Add additional options here
    /*PROTECTED REGION END*/
}
void StandardAlignToPassPos::run(void *msg)
{
    /*PROTECTED REGION ID(run1457532279657) ENABLED START*/ // Add additional options here
    msl::RobotMovement rm;
    msl_actuator_msgs::MotionControl mc;
    auto ownPos = wm->rawSensorData->getOwnPositionVision();
    auto ballPos = wm->ball->getAlloBallPosition();
    if (ownPos == nullptr || ballPos == nullptr)
        return;

    bool ret = query->getSolution(SolverType::GRADIENTSOLVER, runningPlan, result);
    auto passGoal = make_shared<geometry::CNPoint2D>(result[0], result[1]);
    auto p = ballPos + (ballPos - passGoal)->normalize() * (2000.0 / 3.0);
    if (ret)
    {
        msl_helper_msgs::PassMsg pm;
        pm.origin.x = ownPos->x;
        pm.origin.y = ownPos->y;
        pm.destination.x = passGoal->x;
        pm.destination.y = passGoal->y;
        pm.validFor = 5000000000ul;

        // Reduce Communication
        if (iterationCount++ % 3 == 0)
        {
        	//TODO was -1 before
            int tmpID = 0;
            std::vector<uint8_t> robotId;

            for (int i = 0; i < sizeof(int); i++)
            {
                robotId.push_back(*(((uint8_t *)&tmpID) + i));
            }
            send(pm, robotId);
        }
    }

    auto egoBall = wm->ball->getEgoBallPosition();
    if (result.size() > 0)
    {
        auto driveTo = p->alloToEgo(*ownPos);
        // replaced with new moveToPoint method
        //            mc = msl::RobotMovement::placeRobotCareBall(driveTo, egoBall, maxVel);
        movQuery->egoDestinationPoint = driveTo;
        movQuery->egoAlignPoint = ballPos;
        mc = rm.moveToPoint(movQuery);
        if (driveTo->length() < 100)
        {
            mc.motion.translation = 0;
        }
    }
    else
    {
        return;
    }
    send(mc);
    /*PROTECTED REGION END*/
}
void StandardAlignToPassPos::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1457532279657) ENABLED START*/ // Add additional options here
    iterationCount = 0;
    query->clearStaticVariables();
    query->addStaticVariable(getVariablesByName("X"));
    query->addStaticVariable(getVariablesByName("Y"));
    result.clear();
    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1457532279657) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
