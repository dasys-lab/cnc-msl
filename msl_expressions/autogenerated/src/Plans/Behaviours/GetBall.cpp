using namespace std;
#include "Plans/Behaviours/GetBall.h"

/*PROTECTED REGION ID(inccpp1414828300860) ENABLED START*/ //Add additional includes here
#include <tuple>
#include <MSLWorldModel.h>
using namespace std;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1414828300860) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    GetBall::GetBall() :
            DomainBehaviour("GetBall")
    {
        /*PROTECTED REGION ID(con1414828300860) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    GetBall::~GetBall()
    {
        /*PROTECTED REGION ID(dcon1414828300860) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void GetBall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1414828300860) ENABLED START*/ //Add additional options here
        CNPosition ownPos = wm->getOwnPosition();
        CNPoint2D alloBallPos = wm->getBallPosition();
        CNPoint2D egoBallPos = alloBallPos.alloToEgo(ownPos);

        msl_simulator::sim_robot_command c;

        c.velnormal = min(egoBallPos.y * 0.002, 2.0);
        c.veltangent = min(egoBallPos.x * 0.002, 2.0);
        c.velangular = 3 * atan2(egoBallPos.y, egoBallPos.x);

        CNPoint2D p1(0, 1);
        CNPoint2D p2(0, 0);
        p2 = p1.rotate(3.141 * 0.5);

        cout << "GetBall: Point1: (" << p1.x << "," << p1.y << ")" << " Point2: (" << p2.x << "," << p2.y << ")"
                << endl;

        this->send(c);
        /*PROTECTED REGION END*/
    }
    void GetBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1414828300860) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1414828300860) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
