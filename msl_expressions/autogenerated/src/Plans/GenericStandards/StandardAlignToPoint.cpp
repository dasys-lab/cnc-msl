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
<<<<<<< HEAD
        shared_ptr < geometry::CNPoint2D > alloBall = egoBallPos->egoToAllo(*ownPos);

=======

        // return if necessary information is missing
>>>>>>> a15bdeedf6f216132146f89a91075152d997eee2
        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

<<<<<<< HEAD
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
                if (egoTarget->length() < 250 && fabs(egoBallPos->angleTo()) < (M_PI / 180) * 5)
                {
                    this->success = true;
                }
                send(mc);
            }
        }
        else
        {

=======
        // Create allo ball
        shared_ptr < geometry::CNPoint2D > alloBall = egoBallPos->egoToAllo(*ownPos);

        // Create additional points for path planning
        shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
                vector<shared_ptr<geometry::CNPoint2D>>>();
        // add alloBall to path planning
        additionalPoints->push_back(alloBall);

        // robot is executor
        if (!isReceiver)
        {
            // get entry point of task name to locate robot with task name
            EntryPoint* ep = getParentEntryPoint(taskName);
            if (ep != nullptr)
            {
                // get the plan in which the behavior is running
                auto parent = this->runningPlan->getParent().lock();
                if (parent == nullptr)
                {
                    cout << "parent null" << endl;
                    return;
                }
                // get robot ids of robots in found entry point
                shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep);
                shared_ptr < geometry::CNPoint2D > receiverPos = nullptr;
                // exactly one robot is receiver
                int id = ids->at(0);
                if (id != -1)
                {
                    // get receiver position by id
                    auto pos = wm->robots.getTeamMatePosition(id);
                    receiverPos = make_shared < geometry::CNPoint2D > (pos->x, pos->y);
                }
                MotionControl mc;
                shared_ptr < geometry::CNPoint2D > egoTarget = nullptr;
                // if there is a receiver, align to it
                if (receiverPos != nullptr)
                {
                    // calculate target 60cm away from the ball and on a line with the receiver
                    egoTarget = (alloBall + ((alloBall - receiverPos)->normalize() * 600))->alloToEgo(*ownPos);
                    // ask the path planner how to get there
                    mc = RobotMovement::moveToPointCarefully(egoTarget, receiverPos->alloToEgo(*ownPos), 0,
                                                             additionalPoints);
                }
                else
                {
                    // if there is no receiver, align to middle
                    egoTarget = (alloBall + ((alloBall - alloTarget)->normalize() * 600))->alloToEgo(*ownPos);
                    mc = RobotMovement::moveToPointCarefully(egoTarget, alloTarget->alloToEgo(*ownPos), 0,
                                                             additionalPoints);
                }
                // if we reach the point and are aligned, the behavior is successful
                if (egoTarget->length() < 250 && fabs(egoBallPos->rotate(M_PI)->angleTo()) < (M_PI / 180) * 5)
                {
                    this->success = true;
                }
                send(mc);
            }
        }
        else // receiver
        {
            //calculate point on a line with ball and mid on a distance of 2,3m
>>>>>>> a15bdeedf6f216132146f89a91075152d997eee2
            shared_ptr < geometry::CNPoint2D > egoTarget =
                    (alloBall + ((alloBall - alloTarget)->normalize() * -2300))->alloToEgo(*ownPos);

            MotionControl mc;

<<<<<<< HEAD
            mc = RobotMovement::moveToPointCarefully(egoTarget, egoBallPos, 0);

            if (egoTarget->length() < 250 && fabs(egoBallPos->angleTo()) < (M_PI / 180) * 5)
=======
            // ask the path planner how to get there
            mc = RobotMovement::moveToPointCarefully(egoTarget, egoBallPos, 0, additionalPoints);

            // if we reach the point and are aligned, the behavior is successful
            if (egoTarget->length() < 250 && fabs(egoBallPos->rotate(M_PI)->angleTo()) < (M_PI / 180) * 5)
>>>>>>> a15bdeedf6f216132146f89a91075152d997eee2
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
