using namespace std;
#include "Plans/GenericStandards/StandardPass.h"

/*PROTECTED REGION ID(inccpp1435760160067) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "engine/model/EntryPoint.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1435760160067) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    StandardPass::StandardPass() :
            DomainBehaviour("StandardPass")
    {
        /*PROTECTED REGION ID(con1435760160067) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    StandardPass::~StandardPass()
    {
        /*PROTECTED REGION ID(dcon1435760160067) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void StandardPass::run(void* msg)
    {
        /*PROTECTED REGION ID(run1435760160067) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball.getEgoBallPosition();

        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        //TODO
        shared_ptr < geometry::CNPoint2D > egoAlignPoint = nullptr;
        EntryPoint* ep = getParentEntryPoint(taskName);
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
                auto pos = wm->robots.getTeamMatePosition(id);
                egoAlignPoint = make_shared < geometry::CNPoint2D > (pos->x, pos->y);
            }
        }
        else
        {
            shared_ptr < geometry::CNPoint2D > alloAlignPoint = make_shared < geometry::CNPoint2D > (0, 0);
            egoAlignPoint = alloAlignPoint->alloToEgo(*ownPos);
        }

        msl_actuator_msgs::MotionControl mc = msl::RobotMovement::alignToPointWithBall(egoAlignPoint, egoBallPos, 0.005,
                                                                                       0.075);
        if (egoAlignPoint->angleTo() > 0.005)
        {
            send(mc);
        }
        else
        {
            msl_actuator_msgs::KickControl kc;
            kc.enabled = true;
            kc.kicker = 1;
            kc.power = wm->kicker.getKickPowerPass(egoAlignPoint->alloToEgo(*ownPos)->length());
            send(kc);
            this->success = true;
        }

        /*PROTECTED REGION END*/
    }
    void StandardPass::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1435760160067) ENABLED START*/ //Add additional options here
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
            cerr << "Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1435760160067) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */