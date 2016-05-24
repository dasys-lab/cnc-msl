using namespace std;
#include "Plans/Standards/Own/ThrowIn/ReceiveInOppHalf.h"

/*PROTECTED REGION ID(inccpp1462370340143) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "SystemConfig.h"
#include "engine/model/EntryPoint.h"
#include "engine/constraintmodul/ConstraintQuery.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
#include "SolverType.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1462370340143) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    ReceiveInOppHalf::ReceiveInOppHalf() :
            DomainBehaviour("ReceiveInOppHalf")
    {
        /*PROTECTED REGION ID(con1462370340143) ENABLED START*/ //Add additional options here
        this->query = make_shared < alica::ConstraintQuery > (this->wm->getEngine());
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
        shared_ptr < geometry::CNPoint2D > ballPos = wm->ball->getEgoBallPosition();

        if (ownPos == nullptr || ballPos == nullptr)
        {
            return;
        }
        shared_ptr < geometry::CNPoint2D > alloBall = ballPos->egoToAllo(*ownPos);

        MotionControl mc;
        if (query->getSolution(SolverType::GRADIENTSOLVER, runningPlan, result) || result.size() > 1)
        {
            cout << "ReceiveInOppHalf: FOUND a solution!" << endl;
            shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
                    vector<shared_ptr<geometry::CNPoint2D>>>();
            additionalPoints->push_back(alloBall);
            shared_ptr < geometry::CNPoint2D > alloTarget = make_shared < geometry::CNPoint2D
                    > (result.at(0), result.at(1));

            cout << "ReceiveInOppHalf: Target x,y: " << alloTarget->x << " " << alloTarget->y << endl;

            shared_ptr < geometry::CNPoint2D > egoTarget = alloTarget->alloToEgo(*ownPos);

            mc = msl::RobotMovement::moveToPointCarefully(egoTarget, alloBall->alloToEgo(*ownPos), 100.0,
                                                          additionalPoints);
        }
        else
        {
            cout << "ReceiveInOppHalf: Did not get a filled result vector!" << endl;
        }
        send(mc);
        /*PROTECTED REGION END*/
    }
    void ReceiveInOppHalf::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1462370340143) ENABLED START*/ //Add additional options here
    	query->clearDomainVariables();
		query->addVariable(wm->getOwnId(), "x");
		query->addVariable(wm->getOwnId(), "y");
		result.clear();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1462370340143) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
