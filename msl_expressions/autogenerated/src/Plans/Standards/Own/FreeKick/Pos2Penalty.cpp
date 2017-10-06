using namespace std;
#include "Plans/Standards/Own/FreeKick/Pos2Penalty.h"

/*PROTECTED REGION ID(inccpp1465474139420) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "engine/constraintmodul/Query.h"
#include "GSolver.h"
#include "SolverType.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <Robots.h>
using geometry::CNPointAllo;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1465474139420) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Pos2Penalty::Pos2Penalty() :
            DomainBehaviour("Pos2Penalty")
    {
        /*PROTECTED REGION ID(con1465474139420) ENABLED START*/ //Add additional options here
        this->query = make_shared < alica::Query > (this->wm->getEngine());
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
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto ballPos = wm->ball->getPositionEgo();

        if (!ownPos ||!ballPos)
        {
            return;
        }
        auto alloBall = ballPos->toAllo(*ownPos);

        msl::RobotMovement rm;
        msl_actuator_msgs::MotionControl mc;

        if (query->getSolution(SolverType::GRADIENTSOLVER, runningPlan, result) || result.size() > 1)
        {
            cout << "Pos2Penalty: FOUND a solution!" << endl;
            nonstd::optional < vector<geometry::CNPointAllo>> additionalPoints = nonstd::make_optional<
                    vector<geometry::CNPoint2D>>();
            additionalPoints->push_back(alloBall);
            auto alloTarget = CNPointAllo (result.at(0), result.at(1));

            //if solution is inside pen area and another robot is inside pen area already, map solution out of pen area
            if (wm->field->isInsideOppPenalty(alloTarget, 10) && wm->robots->teammates.teammatesInOppPenalty() > 0)
            {
                alloTarget = wm->field->mapOutOfOppPenalty(alloTarget);
            }

            //if solution is inside pen area, make sure not to block path of ball to goal

//			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> trianglePoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
//			trianglePoints->push_back(wm->field->posLeftOppGoalPost());
//			trianglePoints->push_back(wm->field->posRightOppGoalPost());
//			trianglePoints->push_back(alloBall);

//            cout << "Target x,y: " << alloTarget->x << " " << alloTarget->y << endl;

            auto egoTarget = alloTarget.toEgo(*ownPos);

//            moveQuery->fast = false;
            moveQuery.egoDestinationPoint = egoTarget;
            moveQuery.egoAlignPoint = alloBall.toEgo(*ownPos);
            moveQuery.snapDistance = 100.0;
            moveQuery.additionalPoints = additionalPoints;

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
        query->addDomainVariable(wm->getOwnId(), "x");
        query->addDomainVariable(wm->getOwnId(), "y");
        result.clear();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1465474139420) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
