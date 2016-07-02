using namespace std;
#include "Plans/Standards/Own/FreeKick/Pos2Penalty.h"

/*PROTECTED REGION ID(inccpp1465474139420) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "engine/constraintmodul/ConstraintQuery.h"
#include "GSolver.h"
#include "SolverType.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <Robots.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1465474139420) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Pos2Penalty::Pos2Penalty() :
            DomainBehaviour("Pos2Penalty")
    {
        /*PROTECTED REGION ID(con1465474139420) ENABLED START*/ //Add additional options here
        this->query = make_shared < alica::ConstraintQuery > (this->wm->getEngine());
        this->moveQuery = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    Pos2Penalty::~Pos2Penalty()
    {
        /*PROTECTED REGION ID(dcon1465474139420) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Pos2Penalty::run(void* msg)
    {
        /*PROTECTED REGION ID(run1465474139420) ENABLED START*/ //Add additional options here
        auto ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > ballPos = wm->ball->getEgoBallPosition();

        if (ownPos == nullptr || ballPos == nullptr)
        {
            return;
        }
        shared_ptr < geometry::CNPoint2D > alloBall = ballPos->egoToAllo(*ownPos);

        msl::RobotMovement rm;
        msl_actuator_msgs::MotionControl mc;
        if (query->getSolution(SolverType::GRADIENTSOLVER, runningPlan, result) || result.size() > 1)
        {
            cout << "Pos2Penalty: FOUND a solution!" << endl;
            shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
                    vector<shared_ptr<geometry::CNPoint2D>>>();
            additionalPoints->push_back(alloBall);
            shared_ptr < geometry::CNPoint2D > alloTarget = make_shared < geometry::CNPoint2D
                    > (result.at(0), result.at(1));

            //if solution is inside pen area and another robot is inside pen area already, map solution out of pen area
            if (wm->field->isInsideOppPenalty(alloTarget, 10) && wm->robots->teammates.teamMatesInOppPenalty() > 0)
            {
                alloTarget = wm->field->mapOutOfOppPenalty(alloTarget);
            }

            //if solution is inside pen area, make sure not to block path of ball to goal

//			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> trianglePoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
//			trianglePoints->push_back(wm->field->posLeftOppGoalPost());
//			trianglePoints->push_back(wm->field->posRightOppGoalPost());
//			trianglePoints->push_back(alloBall);

            cout << "Target x,y: " << alloTarget->x << " " << alloTarget->y << endl;

            shared_ptr < geometry::CNPoint2D > egoTarget = alloTarget->alloToEgo(*ownPos);

//            moveQuery->fast = false;
            moveQuery->egoDestinationPoint = egoTarget;
            moveQuery->egoAlignPoint = alloBall->alloToEgo(*ownPos);
            moveQuery->snapDistance = 100.0;
            moveQuery->additionalPoints = additionalPoints;

            mc = rm.moveToPoint(moveQuery);

        }
        else
        {
            cout << "Pos2Penalty: Did not get a filled result vector!" << endl;
        }

        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        /*PROTECTED REGION END*/
    }
    void Pos2Penalty::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1465474139420) ENABLED START*/ //Add additional options here
        query->clearDomainVariables();
        query->addVariable(wm->getOwnId(), "x");
        query->addVariable(wm->getOwnId(), "y");
        result.clear();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1465474139420) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
