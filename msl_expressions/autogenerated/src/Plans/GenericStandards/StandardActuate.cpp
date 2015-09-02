using namespace std;
#include "Plans/GenericStandards/StandardActuate.h"

/*PROTECTED REGION ID(inccpp1435766212595) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1435766212595) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    StandardActuate::StandardActuate() :
            DomainBehaviour("StandardActuate")
    {
        /*PROTECTED REGION ID(con1435766212595) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    StandardActuate::~StandardActuate()
    {
        /*PROTECTED REGION ID(dcon1435766212595) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void StandardActuate::run(void* msg)
    {
        /*PROTECTED REGION ID(run1435766212595) ENABLED START*/ //Add additional options here
        msl_actuator_msgs::BallHandleCmd bhc;
        bhc.leftMotor = -wheelSpeed;
        bhc.rightMotor = -wheelSpeed;
        send(bhc);
        /*PROTECTED REGION END*/
    }
    void StandardActuate::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1435766212595) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1435766212595) ENABLED START*/ //Add additional methods here
    void StandardActuate::readConfigParameters()
    {
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        wheelSpeed = (*sc)["Actuation"]->get<double>("Dribble.StdExecGrabBallWheelSpeed", NULL);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
