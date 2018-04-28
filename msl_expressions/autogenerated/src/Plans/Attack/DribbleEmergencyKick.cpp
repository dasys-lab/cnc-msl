using namespace std;
#include "Plans/Attack/DribbleEmergencyKick.h"

/*PROTECTED REGION ID(inccpp1457706800035) ENABLED START*/ //Add additional includes here
#include <RawSensorData.h>
#include <Ball.h>
#include <msl_actuator_msgs/KickControl.h>
#include <MSLWorldModel.h>
#include <SystemConfig.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1457706800035) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DribbleEmergencyKick::DribbleEmergencyKick() :
            DomainBehaviour("DribbleEmergencyKick")
    {
        /*PROTECTED REGION ID(con1457706800035) ENABLED START*/ //Add additional options here
        haveKicked = false;

        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

        this->kickpower = (*sc)["Behaviour"]->get<double>("StandardPass", "DefaultPower", NULL);
        this->safeKick = (*sc)["Behaviour"]->get<bool>("StandardPass", "SafeKick", NULL);

        /*PROTECTED REGION END*/
    }
    DribbleEmergencyKick::~DribbleEmergencyKick()
    {
        /*PROTECTED REGION ID(dcon1457706800035) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DribbleEmergencyKick::run(void* msg)
    {
        /*PROTECTED REGION ID(run1457706800035) ENABLED START*/ //Add additional options here
        ballPos = wm->ball->getAlloBallPosition();
        if (ballPos == nullptr || wm->ball->getBallPickupPosition() == nullptr)
        {
            haveKicked = false;
            return;
        }

        //Console.WriteLine("PickUp Pos: ( "+WM.BallPickupPosition.X + " , " + WM.BallPickupPosition.Y+" )");

        if (!safeKick && wm->ball->getBallPickupPosition()->distanceTo(ballPos) < 2800)
        {
            return;
        }

        msl_actuator_msgs::KickControl km;
        km.enabled = true;

        km.power = (ushort)(330.0 + 0.6 * wm->rawSensorData->getCorrectedOdometryInfo()->motion.translation / 100.0);

        this->setSuccess(true);

        if (!haveKicked)
        {
            send(km);
        }
        haveKicked = true;
        /*PROTECTED REGION END*/
    }
    void DribbleEmergencyKick::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1457706800035) ENABLED START*/ //Add additional options here
        haveKicked = false;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1457706800035) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
