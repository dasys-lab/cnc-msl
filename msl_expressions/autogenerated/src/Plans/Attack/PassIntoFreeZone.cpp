using namespace std;
#include "Plans/Attack/PassIntoFreeZone.h"

/*PROTECTED REGION ID(inccpp1508951632953) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1508951632953) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    PassIntoFreeZone::PassIntoFreeZone() :
            DomainBehaviour("PassIntoFreeZone")
    {
        /*PROTECTED REGION ID(con1508951632953) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    PassIntoFreeZone::~PassIntoFreeZone()
    {
        /*PROTECTED REGION ID(dcon1508951632953) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void PassIntoFreeZone::run(void* msg)
    {
        /*PROTECTED REGION ID(run1508951632953) ENABLED START*/ //Add additional options here
    	/*
    	 * TODO:
    	 * 1. implement constraint for defining pass point
    	 * 2. implement dribbling for passing to pass point (don't forget to send pass message)
    	 * 3. play the pass (chose kickpower wisely)
    	 */
        /*PROTECTED REGION END*/
    }
    void PassIntoFreeZone::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1508951632953) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1508951632953) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
