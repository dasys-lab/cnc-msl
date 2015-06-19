using namespace std;
#include "Plans/GenericStandards/StandardAlignToPoint.h"

/*PROTECTED REGION ID(inccpp1433949970592) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"

/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1433949970592) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    StandardAlignToPoint::StandardAlignToPoint() :
            DomainBehaviour("StandardAlignToPoint")
    {
        /*PROTECTED REGION ID(con1433949970592) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    StandardAlignToPoint::~StandardAlignToPoint()
    {
        /*PROTECTED REGION ID(dcon1433949970592) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void StandardAlignToPoint::run(void* msg)
    {
        /*PROTECTED REGION ID(run1433949970592) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision(); // actually ownPosition corrected
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();

        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        if (!isReceiver)
        {

            shared_ptr < geometry::CNPoint2D > targetPoint = make_shared < geometry::CNPoint2D
                    > (alloTarget.x - egoBallPos->alloToEgo(*ownPos)->x, alloTarget.y
                            - egoBallPos->alloToEgo(*ownPos)->y);

            targetPoint->x = 600 * targetPoint->normalize()->x;
            targetPoint->y = 600 * targetPoint->normalize()->y;

            shared_ptr < geometry::CNPoint2D > egoTarget = targetPoint->alloToEgo(*ownPos);
            MotionControl mc;

            mc = RobotMovement::moveToPointCarefully(egoTarget, egoBallPos, 0);

            if (egoTarget->length() < 250 && fabs(egoBallPos->angleTo()) < 1)
            {
                this->success = true;
            }

            send(mc);

        }
        else
        {

            shared_ptr < geometry::CNPoint2D > targetPoint = make_shared < geometry::CNPoint2D
                    > (alloTarget.x - egoBallPos->alloToEgo(*ownPos)->x, alloTarget.y
                            - egoBallPos->alloToEgo(*ownPos)->y);

            targetPoint->x = -2000 * targetPoint->normalize()->x;
            targetPoint->y = -2000 * targetPoint->normalize()->y;

            shared_ptr < geometry::CNPoint2D > egoTarget = targetPoint->alloToEgo(*ownPos);
            MotionControl mc;

            mc = RobotMovement::moveToPointCarefully(egoTarget, egoBallPos, 0);

            if (egoTarget->length() < 250 && fabs(egoBallPos->angleTo()) < 1)
            {
                this->success = true;
            }

            send(mc);

        }
        /*PROTECTED REGION END*/
    }
    void StandardAlignToPoint::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1433949970592) ENABLED START*/ //Add additional options here
        field = msl::MSLFootballField::getInstance();
        string tmp;
        bool success = true;
        success &= getParameter("X", tmp);
        try
        {
            if (success)
            {
                alloTarget.x = stod(tmp);
            }
            else
            {
                alloTarget.x = field->FieldLength - field->GoalAreaLength;
            }
            success &= getParameter("Y", tmp);
            if (success)
            {
                alloTarget.y = stod(tmp);
            }
            else
            {
                alloTarget.y = 0;
            }
            success &= getParameter("Receiver", tmp);
            if (success)
            {
                isReceiver = (stoi(tmp) != 0);
            }
        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1433949970592) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
