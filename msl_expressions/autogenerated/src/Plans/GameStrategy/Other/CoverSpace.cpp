using namespace std;
#include "Plans/GameStrategy/Other/CoverSpace.h"

/*PROTECTED REGION ID(inccpp1455537892946) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "container/CNPoint2D.h"

using namespace geometry;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1455537892946) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CoverSpace::CoverSpace() :
            DomainBehaviour("CoverSpace")
    {
        /*PROTECTED REGION ID(con1455537892946) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    CoverSpace::~CoverSpace()
    {
        /*PROTECTED REGION ID(dcon1455537892946) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void CoverSpace::run(void* msg)
    {
        /*PROTECTED REGION ID(run1455537892946) ENABLED START*/ //Add additional options here
        auto alloBallPos = wm->ball.getAlloBallPosition();
        if (alloBallPos == nullptr)
        {
            alloBallPos = make_shared < geometry::CNPoint2D > (0, 0);
        }

        auto ownPos = wm->rawSensorData.getOwnPositionVision();
        if (ownPos == nullptr)
        {
            cerr << "No own Position!!!! Initiating Selfdestruction !!!" << endl;
            return;
        }

        //Cover Opposite Side of the field!

        auto alloTarget = alloBallPos->clone();
        if (abs(alloTarget->y) > 1000 || lastPos == nullptr)
        {
            alloTarget->y = -alloTarget->y * positionPercentage;
            lastPos = alloTarget;
        }
        else
        {
            //Hysteresis here!
            //1664 = sqrt(2)+robotradius
            alloTarget->y = lastPos->y / abs(lastPos->y) * 1664;
        }

        //Go closer towards own goal according to position percentage
        alloTarget->x = (alloTarget->x + msl::MSLFootballField::FieldLength / 2) * positionPercentage
                - msl::MSLFootballField::FieldLength / 2;

        //Be sure that the robot is not inside the own penalty area
        if (alloTarget->x < -msl::MSLFootballField::FieldLength / 2 + msl::MSLFootballField::PenaltyAreaLength + 260)
        {
            alloTarget->x = -msl::MSLFootballField::FieldLength / 2 + msl::MSLFootballField::PenaltyAreaLength + 260;
        }

        auto egoTarget = alloTarget->alloToEgo(*ownPos);
        auto egoAlignPoint = alloBallPos->alloToEgo(*ownPos);

        msl_actuator_msgs::MotionControl mc;
        mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoAlignPoint, 100, nullptr);
        send(mc);

        /*PROTECTED REGION END*/
    }
    void CoverSpace::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1455537892946) ENABLED START*/ //Add additional options here
        lastPos.reset();
        string tmp;
        bool success = true;
        success &= getParameter("DefensePercentage", tmp);
        try
        {
            if (success)
            {
                positionPercentage = stod(tmp);
            }
        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (positionPercentage > 1)
            positionPercentage = 1;
        if (positionPercentage < 0.1)
            positionPercentage = 0.1;

        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1455537892946) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
