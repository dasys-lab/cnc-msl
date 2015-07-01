using namespace std;
#include "Plans/GenericStandards/StandardPass.h"

/*PROTECTED REGION ID(inccpp1435760160067) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1435760160067) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    StandardPass::StandardPass() :
            DomainBehaviour("StandardPass")
    {
        /*PROTECTED REGION ID(con1435760160067) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    StandardPass::~StandardPass()
    {
        /*PROTECTED REGION ID(dcon1435760160067) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void StandardPass::run(void* msg)
    {
        /*PROTECTED REGION ID(run1435760160067) ENABLED START*/ //Add additional options here
        shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision();
        shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball.getEgoBallPosition();
        shared_ptr<geometry::CNPoint2D> alloAlignPoint = make_shared<geometry::CNPoint2D>(0,0);
        shared_ptr<geometry::CNPoint2D> egoAlignPoint = alloAlignPoint->alloToEgo(*ownPos);

    	msl::RobotMovement::alignToPointWithBall(egoAlignPoint, egoBallPos, 0.005 ,0.075);

    	/*PROTECTED REGION END*/
    }
    void StandardPass::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1435760160067) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1435760160067) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
