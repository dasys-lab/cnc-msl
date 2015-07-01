using namespace std;
#include "Plans/GenericStandards/StandardAlignToPoint.h"

/*PROTECTED REGION ID(inccpp1433949970592) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "engine/model/EntryPoint.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"

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
        shared_ptr < geometry::CNPoint2D > alloBall = egoBallPos->egoToAllo(*ownPos);

        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        if (!isReceiver)
        {

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
                shared_ptr < geometry::CNPoint2D > receiverPos;
                int id = ids->at(0);
                if (id != -1)
                {
                    auto pos = wm->robots.getTeamMatePosition(id);
                    receiverPos = make_shared < geometry::CNPoint2D > (pos->x, pos->y);
                }
                MotionControl mc;
                shared_ptr < geometry::CNPoint2D > egoTarget = nullptr;
                if (receiverPos != nullptr)
                {
                    egoTarget = (alloBall + ((alloBall - receiverPos)->normalize() * 600))->alloToEgo(*ownPos);
                    mc = RobotMovement::moveToPointCarefully(egoTarget, receiverPos->alloToEgo(*ownPos), 0);
                }
                else
                {
                    egoTarget = (alloBall + ((alloBall - alloTarget)->normalize() * 600))->alloToEgo(*ownPos);
                    mc = RobotMovement::moveToPointCarefully(egoTarget, alloTarget->alloToEgo(*ownPos), 0);
                }
                if (egoTarget->length() < 250 && fabs(egoBallPos->rotate(M_PI)->angleTo()) < (M_PI / 180) * 5)
                {
                    this->success = true;
                }
                send(mc);
            }
        }
        else
        {

            shared_ptr < geometry::CNPoint2D > egoTarget =
                    (alloBall + ((alloBall - alloTarget)->normalize() * -2300))->alloToEgo(*ownPos);

            MotionControl mc;

            mc = RobotMovement::moveToPointCarefully(egoTarget, egoBallPos, 0);

            if (egoTarget->length() < 250 && fabs(egoBallPos->rotate(M_PI)->angleTo()) < (M_PI / 180) * 5)
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
        alloTarget = make_shared < geometry::CNPoint2D > (0, 0);
        success &= getParameter("X", tmp);
        try
        {
            if (success)
            {
                alloTarget->x = stod(tmp);
            }
            else
            {
                alloTarget->x = 0;
            }
            success = true;
            success &= getParameter("Y", tmp);
            if (success)
            {
                alloTarget->y = stod(tmp);
            }
            else
            {
                alloTarget->y = 0;
            }
            success = true;
            success &= getParameter("Receiver", tmp);
            if (success)
            {
                isReceiver = (stoi(tmp) != 0);
            }
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
/*PROTECTED REGION ID(methods1433949970592) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
