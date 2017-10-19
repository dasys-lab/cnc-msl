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
using geometry::CNPointEgo;
using geometry::CNPointAllo;
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
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto egoBallPos = wm->ball->getPositionEgo();

        if (!ownPos || !egoBallPos)
        {
            return;
        }

        CNPointEgo egoAlignPoint;
        EntryPoint* ep = getParentEntryPoint(taskName);
        int id = -1;
        if (ep != nullptr)
        {
            auto parent = this->runningPlan->getParent().lock();
            if (parent == nullptr)
            {
                cout << "parent null" << endl;
                return;
            }
            shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep);
            int id = ids->at(0);
            if (id != -1)
            {
                auto pos = wm->robots->teammates.getTeammatePositionBuffer(id).getLastValidContent();
                if (pos)
                {
                    egoAlignPoint = pos->getPoint().toEgo(*ownPos);
                }
                else
                {
                    egoAlignPoint = CNPointAllo(0, 0).toEgo(*ownPos);
                }
            }
        }
        else
        {
            CNPointAllo alloAlignPoint = CNPointAllo(0, 0);
            egoAlignPoint = alloAlignPoint.toEgo(*ownPos);
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
        auto dest = CNPointEgo(-1, 0);
        dest = dest * egoAlignPoint.length();
        auto alloDest = dest.toAllo(*ownPos);

        pm.destination.x = alloDest.x;
        pm.destination.y = alloDest.y;
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
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "StandardPass: Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1465040441324) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
