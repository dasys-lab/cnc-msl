using namespace std;
#include "Plans/Example/ThaoALignToPointWithBall.h"

/*PROTECTED REGION ID(inccpp1454519765751) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1454519765751) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    ThaoALignToPointWithBall::ThaoALignToPointWithBall() :
            DomainBehaviour("ThaoALignToPointWithBall")
    {
        /*PROTECTED REGION ID(con1454519765751) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    ThaoALignToPointWithBall::~ThaoALignToPointWithBall()
    {
        /*PROTECTED REGION ID(dcon1454519765751) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void ThaoALignToPointWithBall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1454519765751) ENABLED START*/ //Add additional options here
        auto me = wm->rawSensorData.getOwnPositionVision();
        auto egoBallPos = wm->ball.getEgoBallPosition();
        MotionControl mc;
        if (me == nullptr || egoBallPos == nullptr)
        {
            send(mc);
            return;
        }

        mc = msl::RobotMovement::alignToPointWithBall(egoBallPos, egoBallPos, 200, 400);

        send(mc);

        /*PROTECTED REGION END*/
    }
    void ThaoALignToPointWithBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1454519765751) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1454519765751) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
