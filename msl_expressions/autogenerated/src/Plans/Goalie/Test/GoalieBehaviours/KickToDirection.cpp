using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/KickToDirection.h"

/*PROTECTED REGION ID(inccpp1447863478260) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1447863478260) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    KickToDirection::KickToDirection() :
            DomainBehaviour("KickToDirection")
    {
        /*PROTECTED REGION ID(con1447863478260) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    KickToDirection::~KickToDirection()
    {
        /*PROTECTED REGION ID(dcon1447863478260) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void KickToDirection::run(void* msg)
    {
        /*PROTECTED REGION ID(run1447863478260) ENABLED START*/ //Add additional options here
        cout << "### KickToDirection ###" << endl;

        // todo: kick ball away

        cout << "### KickToDirection ###\n" << endl;
        /*PROTECTED REGION END*/
    }
    void KickToDirection::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1447863478260) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1447863478260) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
