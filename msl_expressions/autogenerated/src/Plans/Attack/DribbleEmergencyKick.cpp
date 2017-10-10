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
        this->kickpower = 0;
        this->safeKick = false;
        this->haveKicked = false;
        this->ballPos = nonstd::nullopt;

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
        this->ballPos = wm->ball->getPositionAllo();
        if (!this->ballPos || !wm->ball->getBallPickupPosition())
        {
            haveKicked = false;
            return;
        }

        //Console.WriteLine("PickUp Pos: ( "+WM.BallPickupPosition.X + " , " + WM.BallPickupPosition.Y+" )");

        geometry::CNPointAllo bp = *ballPos;
        if (!safeKick && wm->ball->getBallPickupPosition()->distanceTo(bp) < 2800)
        {
            return;
        }

        msl_actuator_msgs::KickControl km;
        km.enabled = true;
        //TODO nicegeom
        auto odom = wm->rawSensorData->getCorrectedOdometryBuffer().getLastValidContent();
        if (odom)
        {

            km.power = (ushort)(390.0 + 0.6 * odom->motion.translation / 100.0);

            /*List<Point2D> opps = WM.GetOpponentListEgoClustered();
             //If there is an opponent in front of us -> dont do it
             if(opps != null) {
             foreach(Point2D opp in opps) {
             if(Math.Abs(opp.Angle()) > Math.PI-Math.PI/6.0 && opp.Distance() < 1200) {
             return;
             }
             }
             }*/

            this->setSuccess(true);

            if (!haveKicked)
                send(km);
            haveKicked = true;
        }
        /*PROTECTED REGION END*/
    }
    void DribbleEmergencyKick::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1457706800035) ENABLED START*/ //Add additional options here
        haveKicked = false;
        ballPos = nonstd::nullopt;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1457706800035) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
