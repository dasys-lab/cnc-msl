using namespace std;
#include "Plans/GenericStandards/GenericExecutePass.h"

/*PROTECTED REGION ID(inccpp1465040441324) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "engine/model/EntryPoint.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <Robots.h>
#include <msl_robot/kicker/Kicker.h>
#include <msl_helper_msgs/PassMsg.h>
#include <MSLWorldModel.h>
#include <Logger.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1465040441324) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    GenericExecutePass::GenericExecutePass() :
            DomainBehaviour("GenericExecutePass")
    {
        /*PROTECTED REGION ID(con1465040441324) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    GenericExecutePass::~GenericExecutePass()
    {
        /*PROTECTED REGION ID(dcon1465040441324) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void GenericExecutePass::run(void* msg)
    {
        /*PROTECTED REGION ID(run1465040441324) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();

        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        shared_ptr < geometry::CNPoint2D > egoAlignPoint = nullptr;
        EntryPoint* ep = getParentEntryPoint(taskName);
        int id = -1;
        if (ep != nullptr)
        {
            auto parent = this->runningPlan->getParent().lock();
            if (parent == nullptr)
            {
//                cout << "parent null" << endl;
                this->logger->log(this->getName(), "parent null", msl::LogLevels::error);
                return;
            }
            shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep);
            int id = ids->at(0);
            if (id != -1)
            {
                auto pos = wm->robots->teammates.getTeamMatePosition(id);
                egoAlignPoint = pos->getPoint()->alloToEgo(*ownPos);
            }
        }
        else
        {
            shared_ptr < geometry::CNPoint2D > alloAlignPoint = make_shared < geometry::CNPoint2D > (0, 0);
            egoAlignPoint = alloAlignPoint->alloToEgo(*ownPos);
        }

        /*msl_actuator_msgs::MotionControl mc = msl::RobotMovement::alignToPointWithBall(egoAlignPoint, egoBallPos, 0.005,
         0.075);*/
        msl_actuator_msgs::MotionControl mc;
        send(mc);
        msl_actuator_msgs::KickControl kc;
        kc.enabled = true;
        kc.kicker = 1;
        kc.power = 500; //wm->kicker.getKickPowerPass(egoAlignPoint->length());
        send(kc);

        msl_helper_msgs::PassMsg pm;
        pm.validFor = 2000000000ul;
        auto dest = make_shared < geometry::CNPoint2D > (-1, 0);
        dest = dest * egoAlignPoint->length();
        dest = dest->egoToAllo(*ownPos);

        pm.destination.x = dest->x;
        pm.destination.y = dest->y;
        pm.origin.x = ownPos->x;
        pm.origin.y = ownPos->y;
        pm.receiverID = id;
        send(pm);

        this->setSuccess(true);
        /*PROTECTED REGION END*/
    }
    void GenericExecutePass::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1465040441324) ENABLED START*/ //Add additional options here
        string tmp;
        bool success = true;
        try
        {
            success &= getParameter("TaskName", tmp);
            if (success)
            {
                taskName = tmp;
            }
        }
        catch (exception& e)
        {
//            cerr << "Could not cast the parameter properly" << endl;
            this->logger->log(this->getName(), "Could not cast parameter properly", msl::LogLevels::error);
        }
        if (!success)
        {
//            cerr << "StandardPass: Parameter does not exist" << endl;
            this->logger->log(this->getName(), "Parameter does not exist", msl::LogLevels::error);
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1465040441324) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
