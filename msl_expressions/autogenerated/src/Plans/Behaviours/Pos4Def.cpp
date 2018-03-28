using namespace std;
#include "Plans/Behaviours/Pos4Def.h"

/*PROTECTED REGION ID(inccpp1445438142979) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <engine/constraintmodul/Query.h>
#include <GSolver.h>
#include <SolverType.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <Logger.h>
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
        this->mQuery = make_shared<msl::MovementQuery>();
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
        auto ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > ballPos = wm->ball->getEgoBallPosition();

        if (ownPos == nullptr || ballPos == nullptr)
        {
            return;
        }
        shared_ptr < geometry::CNPoint2D > alloBall = ballPos->egoToAllo(*ownPos);

        msl_actuator_msgs::MotionControl mc;
        if (query->getSolution(SolverType::GRADIENTSOLVER, runningPlan, result) || result.size() > 1)
        {
            //cout << "Pos4Def: FOUND a solution!" << endl;
            this->logger->log(this->getName(), "FOUND a solution!", msl::LogLevels::debug);
            shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
                    vector<shared_ptr<geometry::CNPoint2D>>>();
            additionalPoints->push_back(alloBall);
            shared_ptr < geometry::CNPoint2D > alloTarget = make_shared < geometry::CNPoint2D
                    > (result.at(0), result.at(1));

            //cout << "Target x,y: " << alloTarget->x << " " << alloTarget->y << endl;
            this->logger->log(this->getName(), "Target x,y: " + std::to_string(alloTarget->x) + " " + std::to_string(alloTarget->y), msl::LogLevels::debug);

            shared_ptr < geometry::CNPoint2D > egoTarget = alloTarget->alloToEgo(*ownPos);

//            mc = msl::RobotMovement::moveToPointCarefully(egoTarget, alloBall->alloToEgo(*ownPos), 100.0,
//                                                          additionalPoints);
            mQuery->egoDestinationPoint = egoTarget;
            mQuery->egoAlignPoint = alloBall->alloToEgo(*ownPos);
            mQuery->snapDistance = 100;
            mQuery->additionalPoints = additionalPoints;
            mc = this->robot->robotMovement->moveToPoint(mQuery);
        }
        else
        {

            //cout << "Pos4Def: Did not get a filled result vector!" << endl;
        	this->logger->log(this->getName(), "Did not get a filled result vector!", msl::LogLevels::error);
        }
        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        else
        {
            //cout << "Motion command is NaN!" << endl;
            this->logger->log(this->getName(), "Motion command is NaN!", msl::LogLevels::warn);
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
