using namespace std;
#include "Plans/Behaviours/AlignToGoal.h"

/*PROTECTED REGION ID(inccpp1415205272843) ENABLED START*/ //Add additional includes here
#include "GeometryCalculator.h"
#include <Ball.h>
#include <RawSensorData.h>
#include <MSLWorldModel.h>
#include <msl_robot/MSLRobot.h>
#include <MSLFootballField.h>
#include <msl_robot/kicker/Kicker.h>
#include <msl_robot/robotmovement/RobotMovement.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1415205272843) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    AlignToGoal::AlignToGoal() :
            DomainBehaviour("AlignToGoal")
    {
        /*PROTECTED REGION ID(con1415205272843) ENABLED START*/ //Add additional options here
        this->maxVel = 2000;
        this->maxRot = M_PI * 4;
        this->minRot = 0.1;
        this->maxYTolerance = 15;
        this->pRot = 2.1;
        this->dRot = 0.0;
        this->yOffset = 150.0;
        this->lastRotError = 0;
        /*PROTECTED REGION END*/
    }
    AlignToGoal::~AlignToGoal()
    {
        /*PROTECTED REGION ID(dcon1415205272843) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void AlignToGoal::run(void* msg)
    {
        /*PROTECTED REGION ID(run1415205272843) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPoint2D > ballPos = wm->ball->getEgoBallPosition();
        shared_ptr < geometry::CNVelocity2D > ballVel = wm->ball->getEgoBallVelocity();
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < vector<double> > dstscan = wm->rawSensorData->getDistanceScan();

        msl_actuator_msgs::MotionControl mc;
        if (ballPos == nullptr || ownPos == nullptr)
        {
            return;
        }
        if (ballVel == nullptr)
        {
            ballVel = make_shared < geometry::CNVelocity2D > (0, 0);
        }
        else if (ballVel->length() > 5000)
        {
            ballVel = ballVel->normalize() * 5000;
        }
        else
        {
            ballVel = ballVel->clone();
        }

        shared_ptr < geometry::CNPoint2D > aimPoint;
        if (alloAimPoint != nullptr)
        {
            aimPoint = alloAimPoint->alloToEgo(*ownPos);
        }
        else
        {
            aimPoint = this->robot->kicker->getFreeGoalVector(this->yOffset);
            if (aimPoint != nullptr)
            {
                alloAimPoint = aimPoint->egoToAllo(*ownPos);
            }
        }

        if (aimPoint == nullptr)
        {
            this->setFailure(true);
            cout << "AlignToGoal: no aimPoint" << endl;
            return;
        }

        double aimAngle = aimPoint->angleTo();
        double ballAngle = this->robot->kicker->kickerAngle;

        double deltaAngle = -geometry::deltaAngle(aimAngle, ballAngle);
        if (dstscan != nullptr)
        {
            double distBeforeBall = this->robot->kicker->minFree(ballAngle, 200, dstscan);
            if (deltaAngle < 20 * M_PI / 180 && distBeforeBall < 1000)
            {
                cout << "AlignToGoal: failure!" << endl;
                this->setFailure(true);
            }
        }
        mc.motion.rotation = deltaAngle * pRot + (deltaAngle - lastRotError) * dRot;
        double sign = mc.motion.rotation < 0 ? -1 : 1;

        mc.motion.rotation = sign * min(this->maxRot, max(abs(mc.motion.rotation), this->minRot));
        lastRotError = deltaAngle;
        double transBallOrth = ballPos->length() * mc.motion.rotation; //may be negative!
        double transBallTo = max(ballPos->length(), ballVel->length()); //Math.Min(1000,ballVel2.Distance());//
        if (abs(deltaAngle) < 12.0 * M_PI / 180.0)
        {
            transBallTo = max(500.0, transBallTo);
        }

        shared_ptr < geometry::CNPoint2D > driveTo = ballPos->rotate(-M_PI / 2.0);
        driveTo = driveTo->normalize() * transBallOrth;
        driveTo->x += ballPos->normalize()->x * transBallTo;
        driveTo->y += ballPos->normalize()->y * transBallTo;

        if (driveTo->length() > maxVel)
        {
            driveTo = driveTo->normalize() * maxVel;
        }

        mc.motion.angle = driveTo->angleTo();
        mc.motion.translation = driveTo->length();


        auto ruleMC = this->robot->robotMovement->ruleActionForBallGetter();
        if(!std::isnan(ruleMC.motion.translation)) {
        	send(ruleMC);
        } else {
        	sendAndUpdatePT(mc);
        }

        /*PROTECTED REGION END*/
    }
    void AlignToGoal::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1415205272843) ENABLED START*/ //Add additional options here
        //TODO needs to be tested
        this->maxVel = (*this->sc)["Behaviour"]->get<double>("Behaviour", "MaxSpeed", NULL);
        this->maxRot = M_PI * 4;
        this->minRot = 0.1;
        this->maxYTolerance = 15;
        this->pRot = 2.1;
        this->dRot = 0.0;
        this->lastRotError = 0;
        this->yOffset = (*this->sc)["Behaviour"]->get<double>("AlignToGoal", "yOffset", NULL);
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        if (ownPos == nullptr)
        {
            alloAimPoint = nullptr;
        }
        else
        {
            shared_ptr < geometry::CNPoint2D > aimPoint = this->robot->kicker->getFreeGoalVector(this->yOffset);
            if (aimPoint != nullptr)
            {
                alloAimPoint = aimPoint->egoToAllo(*ownPos);
            }
        }
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1415205272843) ENABLED START*/ //Add additional methods here
    double AlignToGoal::goalLineHitPoint(shared_ptr<geometry::CNPosition> ownPos, double egoAngle)
    {

        geometry::CNPoint2D hitVector = geometry::CNPoint2D();
        hitVector.x = cos(egoAngle + ownPos->theta);
        hitVector.y = sin(egoAngle + ownPos->theta);
        double t = (wm->field->getFieldLength() / 2 - ownPos->x) / hitVector.x;
        if (t < 0)
        {
            return numeric_limits<double>::max();
        }
        return ownPos->y + t * hitVector.y;
    }

/*PROTECTED REGION END*/
} /* namespace alica */
