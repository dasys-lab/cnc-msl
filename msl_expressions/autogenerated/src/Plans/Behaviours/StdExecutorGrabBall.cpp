using namespace std;
#include "Plans/Behaviours/StdExecutorGrabBall.h"

/*PROTECTED REGION ID(inccpp1441209011595) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1441209011595) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    StdExecutorGrabBall::StdExecutorGrabBall() :
            DomainBehaviour("StdExecutorGrabBall")
    {
        /*PROTECTED REGION ID(con1441209011595) ENABLED START*/ //Add additional options here
        readConfigParameters();
        /*PROTECTED REGION END*/
    }
    StdExecutorGrabBall::~StdExecutorGrabBall()
    {
        /*PROTECTED REGION ID(dcon1441209011595) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void StdExecutorGrabBall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1441209011595) ENABLED START*/ //Add additional options here
        if (wm->ball.haveBall())
        {
            this->success = true;
            return;
        }
        else
        {
            this->success = false;
        }
        geometry::CNPoint2D alloTarget; // alloTarget= the points that robot has to reach.

        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision();

        // return if necessary information is missing
        if (egoBallPos == nullptr)
        {

            //  (*ownPos).x; ownPos->x smart Pointer

            if (count % 4 == 0)
            {
                alloTarget.x = 3000;
                alloTarget.y = -3000;
            }
            else if (count % 4 == 1)
            {
                alloTarget.x = -3000;
                alloTarget.y = 3000;
            }
            else if (count % 4 == 2)
            {
                alloTarget.x = 3000;
                alloTarget.y = 3000;
            }
            else
            {
                alloTarget.x = 3000;
                alloTarget.y = -3000;
            }

            auto egoTarget = alloTarget.alloToEgo(*ownPos);

            MotionControl mc;
            mc = RobotMovement::moveToPointCarefully(egoTarget, make_shared < geometry::CNPoint2D > (-1000.0, 0.0), 0);

            if (egoTarget->length() < 250)
            {
                count++;
            }

            send(mc);

            //return;

        }
        else
        {
            MotionControl mc = msl::RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, catchRadius, nullptr);

            send(mc);
            /*PROTECTED REGION END*/
        }
    void StdExecutorGrabBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1441209011595) ENABLED START*/ //Add additional options here
        count=0;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1441209011595) ENABLED START*/ //Add additional methods here
    void StdExecutorGrabBall::readConfigParameters()
    {
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        catchRadius = (*sc)["Drive"]->get<double>("Drive.Carefully.CatchRadius", NULL);
    }
    /*PROTECTED REGION END*/
} /* namespace alica */
