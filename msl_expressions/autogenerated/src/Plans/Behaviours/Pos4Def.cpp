using namespace std;
#include "Plans/Behaviours/Pos4Def.h"

/*PROTECTED REGION ID(inccpp1445438142979) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "engine/constraintmodul/Query.h"
#include "GSolver.h"
#include "SolverType.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1445438142979) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Pos4Def::Pos4Def() :
            DomainBehaviour("Pos4Def")
    {
        /*PROTECTED REGION ID(con1445438142979) ENABLED START*/ //Add additional options here
        this->query = make_shared < alica::Query > (this->wm->getEngine());
        /*PROTECTED REGION END*/
    }
    Pos4Def::~Pos4Def()
    {
        /*PROTECTED REGION ID(dcon1445438142979) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Pos4Def::run(void* msg)
    {
        /*PROTECTED REGION ID(run1445438142979) ENABLED START*/ //Add additional options here
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto ballPos = wm->ball->getPositionEgo();

        if (!ownPos|| !ballPos)
        {
            return;
        }
        auto alloBall = ballPos->toAllo(*ownPos);

        msl_actuator_msgs::MotionControl mc;
        if (query->getSolution(SolverType::GRADIENTSOLVER, runningPlan, result) || result.size() > 1)
        {
            cout << "Pos4Def: FOUND a solution!" << endl;
            auto additionalPoints = vector<geometry::CNPointAllo>();
            additionalPoints.push_back(alloBall);
            auto alloTarget = geometry::CNPointAllo(result.at(0), result.at(1));

            cout << "Target x,y: " << alloTarget.x << " " << alloTarget.y << endl;

            geometry::CNPointEgo egoTarget = alloTarget.toEgo(*ownPos);

//            mc = msl::RobotMovement::moveToPointCarefully(egoTarget, alloBall->alloToEgo(*ownPos), 100.0,
//                                                          additionalPoints);
            msl::RobotMovement rm;
            mQuery.egoDestinationPoint = egoTarget;
            mQuery.egoAlignPoint = alloBall.toEgo(*ownPos);
            mQuery.snapDistance = 100;
            mQuery.additionalPoints = additionalPoints;
            mc = rm.moveToPoint(mQuery);
        }
        else
        {

            cout << "Pos4Def: Did not get a filled result vector!" << endl;
        }
        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        else
        {
            cout << "Motion command is NaN!" << endl;
        }
        /*PROTECTED REGION END*/
    }
    void Pos4Def::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1445438142979) ENABLED START*/ //Add additional options here
        query->clearDomainVariables();
        query->addDomainVariable(wm->getOwnId(), "x");
        query->addDomainVariable(wm->getOwnId(), "y");
        result.clear();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1445438142979) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
