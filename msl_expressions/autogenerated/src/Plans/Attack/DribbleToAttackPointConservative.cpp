using namespace std;
#include "Plans/Attack/DribbleToAttackPointConservative.h"

/*PROTECTED REGION ID(inccpp1458132872550) ENABLED START*/ //Add additional includes here
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLFootballField.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1458132872550) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DribbleToAttackPointConservative::DribbleToAttackPointConservative() :
            DomainBehaviour("DribbleToAttackPointConservative")
    {
        /*PROTECTED REGION ID(con1458132872550) ENABLED START*/ //Add additional options here
        currentTarget = make_shared<geometry::CNPoint2D>();
        attackPosY.push_back(wm->field->getFieldWidth() / 3.0 - 700);
        attackPosY.push_back(0);
        attackPosY.push_back(-wm->field->getFieldWidth() / 3.0 + 700);
        query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    DribbleToAttackPointConservative::~DribbleToAttackPointConservative()
    {
        /*PROTECTED REGION ID(dcon1458132872550) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DribbleToAttackPointConservative::run(void* msg)
    {
        /*PROTECTED REGION ID(run1458132872550) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;

        auto ownPos = wm->rawSensorData->getOwnPositionVision();
        auto ballPos = wm->ball->getEgoBallPosition();
        auto dstscan = wm->rawSensorData->getDistanceScan();

        if (ownPos == nullptr)
        {
            return;
        }
        if (currentTarget == nullptr)
        {
            trueInitialize();
        }
        if (currentTarget == nullptr)
        {
            return;
        }
        auto egoTarget = currentTarget->alloToEgo(*ownPos);
        if (egoTarget->length() < 1200)
        {
            this->setSuccess(true);
        }

        query->egoDestinationPoint = egoTarget;

        auto bm = rm.moveToPoint(query);

        auto tmpMC = rm.ruleActionForBallGetter();
        if (!std::isnan(tmpMC.motion.translation))
        {
            send(tmpMC);
        }
        else if (!std::isnan(bm.motion.translation))
        {
            send(bm);
        } else {

        }
        /*PROTECTED REGION END*/
    }
    void DribbleToAttackPointConservative::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1458132872550) ENABLED START*/ //Add additional options here
        currentTarget = nullptr;
//        msl::RobotMovement::reset();
        trueInitialize();
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1458132872550) ENABLED START*/ //Add additional methods here
    void DribbleToAttackPointConservative::trueInitialize() // so true and so evil
    {
        auto ownPos = wm->rawSensorData->getOwnPositionVision();
        if (ownPos == nullptr)
        {
            return;
        }
//		Random rand = new Random();
//		int index = (int)Math.Round(rand.NextDouble()*2.0);

        srand(supplementary::DateTime::getUtcNowC());
        int index = (rand() % attackPosY.size());

        if (ownPos->x < wm->field->getFieldLength() / 6.0)
        {
            currentTarget = make_shared < geometry::CNPoint2D > (wm->field->getFieldLength() / 6.0 - 1500, 0);
            //} else if (ownPos.X < field.FieldLength/2.0) {
            //	currentTarget = new Point2D(field.FieldLength/2.0,0);
        }
        else
        {
            currentTarget = make_shared < geometry::CNPoint2D > (wm->field->getFieldLength() / 4.0 - 1500, 0);
        }
        currentTarget->y = attackPosY.at(index);
        if (currentTarget->alloToEgo(*ownPos)->length() < 1500)
        {
            index = (index + 1) % attackPosY.size(); // select next point in vector
            currentTarget->y = attackPosY.at(index);
        }
    }
/*PROTECTED REGION END*/
} /* namespace alica */
