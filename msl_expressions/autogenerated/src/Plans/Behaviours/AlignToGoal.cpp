using namespace std;
#include "Plans/Behaviours/AlignToGoal.h"

/*PROTECTED REGION ID(inccpp1415205272843) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1415205272843) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    AlignToGoal::AlignToGoal() :
            DomainBehaviour("AlignToGoal")
    {
        /*PROTECTED REGION ID(con1415205272843) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    AlignToGoal::~AlignToGoal()
    {
        /*PROTECTED REGION ID(dcon1415205272843) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void AlignToGoal::run(void* msg)
    {
        /*PROTECTED REGION ID(run1415205272843) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision();
        geometry::CNPoint2D alloEnemyGoal = geometry::CNPoint2D(-3000, 0);
        geometry::CNPoint2D egoEnemyGoal = *alloEnemyGoal.alloToEgo(*ownPos);

        /*  msl_simulator::sim_robot_command c;

         c.velnormal = 3 * atan2(egoEnemyGoal.y, egoEnemyGoal.x);*/

        /*PROTECTED REGION END*/
    }
    void AlignToGoal::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1415205272843) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1415205272843) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
