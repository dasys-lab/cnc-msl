using namespace std;
#include "Plans/Goalie/Test/GoalieBehaviours/DriveToGoal.h"

/*PROTECTED REGION ID(inccpp1447863424939) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
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
        goalInitPos = (*this->sc)["Behaviour"]->get < string > ("Goalie.GoalInitPosition", NULL);
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
        msl::RobotMovement rm;

        cout << "### DriveToGoal ###" << endl;
        shared_ptr < geometry::CNPosition > me;
        double alloTargetX, alloTargetY;

        me = wm->rawSensorData->getOwnPositionVision();

        if (me == nullptr)
        {
            mc.motion.angle = 0;
            mc.motion.rotation = 0;
            mc.motion.translation = 0;

            cout << " [DriveToGoal] Stop!" << endl;
            cout << "### DriveToGoal ###\n" << endl;
        }
        else
        {
            if (goalInitPos.compare("Left") == 0)
            {
                alloTargetX = alloGoalLeft->x - 100;
                alloTargetY = alloGoalLeft->y;
            }
            else if (goalInitPos.compare("Right") == 0)
            {
                alloTargetX = alloGoalRight->x - 100;
                alloTargetY = alloGoalRight->y;
            }
            else
            {
                alloTargetX = alloGoalMid->x - 100;
                alloTargetY = alloGoalMid->y;
            }

            alloTarget = make_shared < geometry::CNPoint2D > (alloTargetX, alloTargetY);
            alloFieldCenterAlignPoint = wm->field->posCenterMarker();

            cout << " Driving to goal" << endl;
            // replaced with new moveToPoint method
//            mc = msl::RobotMovement::moveToPointCarefully(alloTarget->alloToEgo(*me),
//                                                          alloFieldCenterAlignPoint->alloToEgo(*me), 100, 0);
            query->egoDestinationPoint = alloTarget->alloToEgo(*me);
            query->egoAlignPoint = alloFieldCenterAlignPoint->alloToEgo(*me);
            query->snapDistance = 100;

            mc = rm.moveToPoint(query);

            if (me->distanceTo(alloTarget) <= 100)
            {
                msl_actuator_msgs::MotionControl mcStop;
                mcStop.motion.translation = 0;
                mcStop.motion.rotation = 0;
                mcStop.motion.angle = 0;
                send(mcStop);
                this->setSuccess(true);
            }
            else if (!std::isnan(mc.motion.translation))
            {
                cout << "Distance left: " << me->distanceTo(alloTarget) << endl;
                send (mc);
            }
            else
            {
                cout << "Motion command is NaN!" << endl;
            }
            cout << "### DriveToGoal ###\n" << endl;
        }
        /*PROTECTED REGION END*/
    }
    void DriveToGoal::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1447863424939) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1447863424939) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
