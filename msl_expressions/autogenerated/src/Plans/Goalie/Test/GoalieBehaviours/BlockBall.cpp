using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/BlockBall.h"

/*PROTECTED REGION ID(inccpp1447863456983) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1447863456983) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    BlockBall::BlockBall() :
            DomainBehaviour("BlockBall")
    {
        /*PROTECTED REGION ID(con1447863456983) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    BlockBall::~BlockBall()
    {
        /*PROTECTED REGION ID(dcon1447863456983) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void BlockBall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1447863456983) ENABLED START*/ //Add additional options here
        // TODO:
        //		send kick message to block incoming ball
        //		kick can only be executed if 4seconds have passed from last kick
        //		use timestamp of messages
        /*PROTECTED REGION END*/
    }
    void BlockBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1447863456983) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1447863456983) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
