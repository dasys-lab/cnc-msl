using namespace std;
#include "Plans/Standards/Own/ThrowIn/ReceiveInOppHalf.h"

/*PROTECTED REGION ID(inccpp1462370340143) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <SystemConfig.h>
#include <engine/model/EntryPoint.h>
#include <engine/constraintmodul/Query.h>
#include <engine/RunningPlan.h>
#include <engine/Assignment.h>
#include <engine/model/Plan.h>
#include <SolverType.h>
#include <RawSensorData.h>
#include <MSLWorldModel.h>
#include <Ball.h>
#include <Robots.h>
#include <pathplanner/PathPlanner.h>
#include <Logger.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1462370340143) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    ReceiveInOppHalf::ReceiveInOppHalf() :
            DomainBehaviour("ReceiveInOppHalf")
    {
        /*PROTECTED REGION ID(con1462370340143) ENABLED START*/ //Add additional options here
        this->query = make_shared < alica::Query > (this->wm->getEngine());
        this->mQuery = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    ReceiveInOppHalf::~ReceiveInOppHalf()
    {
        /*PROTECTED REGION ID(dcon1462370340143) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void ReceiveInOppHalf::run(void* msg)
    {
        /*PROTECTED REGION ID(run1462370340143) ENABLED START*/ //Add additional options here
        auto ownPos = wm->rawSensorData->getOwnPositionVision();
        auto alloBallPose = wm->ball->getAlloBallPosition();
        if (!ownPos || !alloBallPose)
            return;

        double yCordOfReceiver = 0.0;
        double securityReceiver = 40.0;
        if (alloBallPose->y < 0.0) // right side line
        {
            // place the receiver 1m outside the sideline
//            if (wm->field->getSurrounding() > 1300)
//            {
//                yCordOfReceiver = -wm->field->getFieldWidth() / 2.0 - 1000.0;
//            }
//            else
//            {
//                yCordOfReceiver = -wm->field->getFieldWidth() / 2.0 - wm->field->getSurrounding()
//                        + wm->pathPlanner->getRobotRadius() + securityReceiver;
//            }

            // ^ forbidden with rule changes 2017. place robot on side line instead
            yCordOfReceiver = -wm->field->getFieldWidth() / 2.0 + securityReceiver;
        }
        else // left side line
        {
            // place the receiver 1m outside the sideline
//			if (wm->field->getSurrounding() > 1300)
//			{
//				yCordOfReceiver = wm->field->getFieldWidth() / 2.0 + 1000.0;
//			}
//			else
//			{
//				yCordOfReceiver = wm->field->getFieldWidth() / 2.0 + wm->field->getSurrounding()
//						- wm->pathPlanner->getRobotRadius() - securityReceiver;
//			}

            // ^ forbidden with rule changes 2017. place robot on side line instead

            yCordOfReceiver = wm->field->getFieldWidth() / 2.0 - securityReceiver;
        }

        double lowestX = wm->field->getFieldLength() / 2;
        auto opps = wm->robots->opponents.getOpponentsAlloClustered();
        for (auto opp : *opps)
        {
            double distToLine = geometry::distancePointToLineSegment(opp->x, opp->y, 2000, yCordOfReceiver,
                                                                     wm->field->getFieldLength() / 2 - 2000,
                                                                     yCordOfReceiver);
            if (distToLine > 3000)
            {
                continue;
            }

            if (lowestX > opp->x)
            {
                lowestX = opp->x;
            }
        }

        auto alloTarget = make_shared < geometry::CNPoint2D > (wm->field->getFieldLength() / 4, yCordOfReceiver);

        if (lowestX < wm->field->getFieldLength() / 2 - 2000)
        { // opponent close to pass line
            alloTarget->x = min(alloTarget->x, max(lowestX - 2000, 2000.0));
        }

        // Create additional points for path planning
        shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
                vector<shared_ptr<geometry::CNPoint2D>>>();

        // add alloBall to path planning
        additionalPoints->push_back(alloBallPose);
//        msl_actuator_msgs::MotionControl mc = msl::RobotMovement::moveToPointCarefully(alloTarget->alloToEgo(*ownPos),
//                                                                                       alloBallPose->alloToEgo(*ownPos),
//                                                                                       100.0, additionalPoints);
        mQuery->egoDestinationPoint = alloTarget->alloToEgo(*ownPos);
        mQuery->egoAlignPoint = alloBallPose->alloToEgo(*ownPos);
        mQuery->snapDistance = 100;
        mQuery->additionalPoints = additionalPoints;
        msl_actuator_msgs::MotionControl mc = this->robot->robotMovement->moveToPoint(mQuery);

        send(mc);
        /*PROTECTED REGION END*/
    }
    void ReceiveInOppHalf::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1462370340143) ENABLED START*/ //Add additional options here
        query->clearDomainVariables();
        query->addDomainVariable(wm->getOwnId(), "x");
        query->addDomainVariable(wm->getOwnId(), "y");
        result.clear();
        string tmp;
        bool success = true;
        alloTarget = make_shared < geometry::CNPoint2D > (0, 0);
        try
        {
            success &= getParameter("TeamMateTaskName", tmp);
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
//            cerr << "PRT: Parameter does not exist" << endl;
        	this->logger->log(this->getName(), "Parameter does not exist", msl::LogLevels::error);
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1462370340143) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
