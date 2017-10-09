using namespace std;
#include "Plans/Attack/CatchPass.h"

/*PROTECTED REGION ID(inccpp1440754525537) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <WhiteBoard.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1440754525537) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CatchPass::CatchPass() :
            DomainBehaviour("CatchPass")
    {
        /*PROTECTED REGION ID(con1440754525537) ENABLED START*/ //Add additional options here
//        this->maxVel = (*this->sc)["Behaviour"]->get<double>("Behaviour.MaxSpeed", NULL);
        field = nullptr;
        maxVel = 0;
        sc = nullptr;

        /*PROTECTED REGION END*/
    }
    CatchPass::~CatchPass()
    {
        /*PROTECTED REGION ID(dcon1440754525537) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void CatchPass::run(void* msg)
    {
        /*PROTECTED REGION ID(run1440754525537) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;

        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        if (!ownPos)
        {
            return;
        }

        auto pm = wm->whiteBoard->getPassMsgBuffer().getLastValidContent();
        msl_actuator_msgs::MotionControl mc;
        mc.senderID = -1;
        if (pm)
        { // next go for pass msg
            passOrigin = geometry::CNPointAllo(pm->origin.x, pm->origin.y);
            passDestination = geometry::CNPointAllo(pm->destination.x, pm->destination.y);
            passVector = geometry::CNVecAllo (pm->destination.x, pm->destination.y) - passOrigin;
            auto ballPos = wm->ball->getPositionEgo();
            if (!ballPos)
            {
                ballPos = nonstd::make_optional<geometry::CNPointEgo>(passOrigin.toEgo(*ownPos));
            }

            auto egoDest = passDestination.toEgo(*ownPos);

            double error = egoDest.length();
            double trans = error * 3.0;
            if (error > 2000)
            {
                trans = maxVel;
            }
            trans = min(maxVel, trans);
            // replaced with new moveToPoint method
//            mc = msl::RobotMovement::moveToPointCarefully(egoDest, ballPos, 100);
            query.egoDestinationPoint = egoDest;
            query.egoAlignPoint = ballPos;
            query.snapDistance = 100;
            mc = rm.moveToPoint(query);

            mc.motion.translation = min(mc.motion.translation, trans);

            if (egoDest.length() < 100)
            {
                mc.motion.translation = 0;
            }

        }

        if (!std::isnan(mc.motion.rotation))
        {
            send(mc);
        }
        else
        {
            cout << "motion command is NaN" << endl;
        }
        /*PROTECTED REGION END*/
    }
    void CatchPass::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1440754525537) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1440754525537) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
