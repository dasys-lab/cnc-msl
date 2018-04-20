using namespace std;
#include "Plans/Behaviours/Pos4Def.h"

/*PROTECTED REGION ID(inccpp1445438142979) ENABLED START*/ // Add additional includes here
#include <Ball.h>
#include <GSolver.h>
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <SolverType.h>
#include <engine/constraintmodul/Query.h>
#include <msl_robot/MSLRobot.h>
#include <msl_robot/robotmovement/MovementQuery.h>
#include <msl_robot/robotmovement/RobotMovement.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1445438142979) ENABLED START*/ // initialise static variables here
    /*PROTECTED REGION END*/
    Pos4Def::Pos4Def() :
            DomainBehaviour("Pos4Def")
    {
        /*PROTECTED REGION ID(con1445438142979) ENABLED START*/ // Add additional options here
        this->query = make_shared < alica::Query > (this->wm->getEngine());
        this->mQuery = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    Pos4Def::~Pos4Def()
    {
        /*PROTECTED REGION ID(dcon1445438142979) ENABLED START*/ // Add additional options here
        /*PROTECTED REGION END*/
    }
    void Pos4Def::run(void* msg)
    {
        /*PROTECTED REGION ID(run1445438142979) ENABLED START*/ // Add additional options here
        auto ownPos = this->wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > ballPos = wm->ball->getEgoBallPosition();

        if (ownPos == nullptr || ballPos == nullptr)
        {
            return;
        }
        shared_ptr < geometry::CNPoint2D > alloBall = ballPos->egoToAllo(*ownPos);

        msl_actuator_msgs::MotionControl mc;
        if (query->getSolution(SolverType::GRADIENTSOLVER, runningPlan, result) || result.size() > 1)
        {
            shared_ptr < geometry::CNPoint2D > alloTarget = make_shared < geometry::CNPoint2D
                    > (result.at(0), result.at(1));

            cout << "Target x,y: " << alloTarget->x << " " << alloTarget->y << endl;

            shared_ptr < geometry::CNPoint2D > egoTarget = alloTarget->alloToEgo(*ownPos);

            this->mQuery->egoDestinationPoint = egoTarget;
            this->mQuery->egoAlignPoint = ballPos;
            this->mQuery->snapDistance = 100.0;

            shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
                    vector<shared_ptr<geometry::CNPoint2D>>>();
            additionalPoints->push_back(alloBall);
            this->mQuery->additionalPoints = additionalPoints;

            mc = this->robot->robotMovement->moveToPoint(this->mQuery);
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
        /*PROTECTED REGION ID(initialiseParameters1445438142979) ENABLED START*/ // Add additional options here
        query->clearDomainVariables();
        query->addDomainVariable(wm->getOwnId(), "x");
        query->addDomainVariable(wm->getOwnId(), "y");
        result.clear();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1445438142979) ENABLED START*/ // Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
