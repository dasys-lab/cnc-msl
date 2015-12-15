using namespace std;
#include "Plans/Behaviours/Duel.h"

/*PROTECTED REGION ID(inccpp1450178699265) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1450178699265) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Duel::Duel() :
            DomainBehaviour("Duel")
    {
        /*PROTECTED REGION ID(con1450178699265) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    Duel::~Duel()
    {
        /*PROTECTED REGION ID(dcon1450178699265) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Duel::run(void* msg)
    {
        /*PROTECTED REGION ID(run1450178699265) ENABLED START*/ //Add additional options here

        shared_ptr < geometry::CNPosition > me = wm->rawSensorData.getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();

        if (me == nullptr || egoBallPos == nullptr)
        {
            return;
        }


        /*PROTECTED REGION END*/
    }
    void Duel::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1450178699265) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1450178699265) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
