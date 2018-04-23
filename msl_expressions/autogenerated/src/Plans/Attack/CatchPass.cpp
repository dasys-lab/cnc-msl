using namespace std;
#include "Plans/Attack/CatchPass.h"

/*PROTECTED REGION ID(inccpp1440754525537) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/robotmovement/MovementQuery.h>
#include <msl_robot/MSLRobot.h>
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
        this->maxVel = (*this->sc)["Behaviour"]->get<double>("Behaviour.MaxSpeed", NULL);
        this->query = make_shared<msl::MovementQuery>();
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
        auto ownPos = wm->rawSensorData->getOwnPositionVision();
        if (ownPos == nullptr)
        {
            return;
        }

        shared_ptr < msl_helper_msgs::PassMsg > pm = wm->whiteBoard->getPassMsg();
        msl_actuator_msgs::MotionControl mc;
        mc.senderID = -1;
        if (pm != nullptr)
        { // next go for pass msg
            passOrigin = make_shared < geometry::CNPoint2D > (pm->origin.x, pm->origin.y);
            passDestination = make_shared < geometry::CNPoint2D > (pm->destination.x, pm->destination.y);
            passVector = make_shared < geometry::CNPoint2D > (pm->destination.x, pm->destination.y) - passOrigin;
            shared_ptr < geometry::CNPoint2D > ballPos = wm->ball->getEgoBallPosition();
            if (ballPos == nullptr)
            {
                ballPos = passOrigin->alloToEgo(*ownPos);
            }

            shared_ptr < geometry::CNPoint2D > egoDest = passDestination->alloToEgo(*ownPos);

            double error = egoDest->length();
            double trans = error * 3.0;
            if (error > 2000)
            {
                trans = maxVel;
            }
            trans = min(maxVel, trans);
            query->egoDestinationPoint = egoDest;
            query->egoAlignPoint = ballPos;
            query->snapDistance = 100;
            mc = this->robot->robotMovement->moveToPoint(query);

            mc.motion.translation = min(mc.motion.translation, trans);

            if (egoDest->length() < 100)
            {
                mc.motion.translation = 0;
            }

        }

        if (!std::isnan(mc.motion.rotation))
        {
            sendAndUpdatePT(mc);
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
        passOrigin = nullptr;
        passDestination = nullptr;
        passVector = nullptr;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1440754525537) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
