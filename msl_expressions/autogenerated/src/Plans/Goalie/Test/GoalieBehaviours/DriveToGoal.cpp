using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/DriveToGoal.h"

/*PROTECTED REGION ID(inccpp1447863424939) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <container/CNPoint2D.h>
#include <RawSensorData.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1447863424939) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DriveToGoal::DriveToGoal() :
            DomainBehaviour("DriveToGoal")
    {
        /*PROTECTED REGION ID(con1447863424939) ENABLED START*/ //Add additional options here
        goalieSize = (*this->sc)["Behaviour"]->get<int>("Goalie.GoalieSize", NULL);
        alloGoalMid = wm->field->posOwnGoalMid();

        alloGoalLeft = make_shared < geometry::CNPoint2D
                > (alloGoalMid->x, wm->field->posLeftOwnGoalPost()->y - goalieSize / 2);
        alloGoalRight = make_shared < geometry::CNPoint2D
                > (alloGoalMid->x, wm->field->posRightOwnGoalPost()->y + goalieSize / 2);

        query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    DriveToGoal::~DriveToGoal()
    {
        /*PROTECTED REGION ID(dcon1447863424939) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DriveToGoal::run(void* msg)
    {
        /*PROTECTED REGION ID(run1447863424939) ENABLED START*/ //Add additional options here
        auto me = wm->rawSensorData->getOwnPositionVision();

        if (me == nullptr)
        {
            mc.motion.angle = 0;
            mc.motion.rotation = 0;
            mc.motion.translation = 0;
            send (mc);
            return;
        }

        auto offset = std::make_shared < geometry::CNPoint2D > (200, 0);
        alloTarget = alloGoalMid + offset;

        // Build MostionControl using query
        const double snapDistance = 300;
        query->egoDestinationPoint = alloTarget->alloToEgo(*me);
        query->egoAlignPoint = alloTarget->alloToEgo(*me);
        query->snapDistance = snapDistance;
        // Add goal posts as obstacles
        auto additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        additionalPoints->push_back(alloGoalLeft);
        additionalPoints->push_back(alloGoalRight);
        // Add ball as obstacle
        auto alloBall = wm->ball->getAlloBallPosition();
        if (alloBall != nullptr)
            additionalPoints->push_back(alloBall);
        query->additionalPoints = additionalPoints;
        mc = this->robot->robotMovement->moveToPoint(query);

        // TODO: Probably remove as soon as motion is fixed
        // Clamp translation because of motion failure
        if (mc.motion.translation > 1500)
        {
            mc.motion.translation = 1500;
        }

        if (me->distanceTo(alloTarget) <= snapDistance)
        {
            mc.motion.translation = 0;
            mc.motion.rotation = 0;
            mc.motion.angle = 0;
            send (mc);
            cout << "Goal reached!" << endl;
            this->setSuccess(true);
            return;
        }
        if (std::isnan(mc.motion.translation))
        {
            return;
        }
        // cout << "DriveToGoal: Distance left: " << me->distanceTo(alloTarget) << endl;
        send (mc);
        /*PROTECTED REGION END*/
    }
    void DriveToGoal::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1447863424939) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1447863424939) ENABLED START*/ //Add additional methods here
    void DriveToGoal::updateGoalPosition()
    {
        shared_ptr < geometry::CNPoint2D > laserDetectedEgoGoalMid = wm->rawSensorData->getEgoGoalMid();

        if (laserDetectedEgoGoalMid)
        {
            alloGoalMid = laserDetectedEgoGoalMid;
        }
        else
        {
            alloGoalMid = wm->field->posOwnGoalMid();
        }
        if (alloGoalMid == nullptr || wm->field->posLeftOwnGoalPost() == nullptr)
        {
            cout << "Can't determine goal mid using scanner, alloGoalMid == nullptr" << endl;
            return;
        }
        alloGoalLeft = make_shared < geometry::CNPoint2D
                > (alloGoalMid->x, wm->field->posLeftOwnGoalPost()->y - goalieSize / 2);
        alloGoalRight = make_shared < geometry::CNPoint2D
                > (alloGoalMid->x, wm->field->posRightOwnGoalPost()->y + goalieSize / 2);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
