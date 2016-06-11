using namespace std;
#include "Plans/Behaviours/StdExecutorGrabBall.h"

/*PROTECTED REGION ID(inccpp1441209011595) ENABLED START*/ //Add additional includes here
#include <Ball.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1441209011595) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    StdExecutorGrabBall::StdExecutorGrabBall() :
            DomainBehaviour("StdExecutorGrabBall")
    {
        /*PROTECTED REGION ID(con1441209011595) ENABLED START*/ //Add additional options here
        readConfigParameters();
        /*PROTECTED REGION END*/
    }
    StdExecutorGrabBall::~StdExecutorGrabBall()
    {
        /*PROTECTED REGION ID(dcon1441209011595) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void StdExecutorGrabBall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1441209011595) ENABLED START*/ //Add additional options here
        if (wm->ball->haveBall())
        {
            this->setSuccess(true);
            return;
        }
        else
        {
            this->setSuccess(false);
        }

        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();

        // return if necessary information is missing
        if (egoBallPos == nullptr)
        {
            return;
        }

        msl_actuator_msgs::MotionControl mc = msl::RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos,
                                                                                       catchRadius, nullptr);

        send(mc);
        /*PROTECTED REGION END*/
    }
    void StdExecutorGrabBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1441209011595) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1441209011595) ENABLED START*/ //Add additional methods here
    void StdExecutorGrabBall::readConfigParameters()
    {
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        catchRadius = (*sc)["Drive"]->get<double>("Drive.Carefully.CatchRadius", NULL);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
