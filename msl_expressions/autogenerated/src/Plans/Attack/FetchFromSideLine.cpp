using namespace std;
#include "Plans/Attack/FetchFromSideLine.h"

/*PROTECTED REGION ID(inccpp1450175655102) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1450175655102) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    FetchFromSideLine::FetchFromSideLine() :
            DomainBehaviour("FetchFromSideLine")
    {
        /*PROTECTED REGION ID(con1450175655102) ENABLED START*/ //Add additional options here
		threshold = 400;
		behindDistance = 300;
		maxVel = 3000;
        /*PROTECTED REGION END*/
    }
    FetchFromSideLine::~FetchFromSideLine()
    {
        /*PROTECTED REGION ID(dcon1450175655102) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void FetchFromSideLine::run(void* msg)
    {
        /*PROTECTED REGION ID(run1450175655102) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void FetchFromSideLine::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1450175655102) ENABLED START*/ //Add additional options here

		field = msl::MSLFootballField::getInstance();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1450175655102) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
