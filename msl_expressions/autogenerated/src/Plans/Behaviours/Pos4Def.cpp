using namespace std;
#include "Plans/Behaviours/Pos4Def.h"

/*PROTECTED REGION ID(inccpp1445438142979) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1445438142979) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Pos4Def::Pos4Def() :
            DomainBehaviour("Pos4Def")
    {
        /*PROTECTED REGION ID(con1445438142979) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    Pos4Def::~Pos4Def()
    {
        /*PROTECTED REGION ID(dcon1445438142979) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Pos4Def::run(void* msg)
    {
        /*PROTECTED REGION ID(run1445438142979) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();
        /*PROTECTED REGION END*/
    }
    void Pos4Def::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1445438142979) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1445438142979) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
