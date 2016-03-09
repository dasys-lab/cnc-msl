using namespace std;
#include "Plans/GenericBehaviours/CheckGoalKick.h"

/*PROTECTED REGION ID(inccpp1449076008755) ENABLED START*/ //Add additional includes here
#include <GeometryCalculator.h>
#include <math.h>

#include <Rules.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1449076008755) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CheckGoalKick::CheckGoalKick() :
            DomainBehaviour("CheckGoalKick")
    {
        /*PROTECTED REGION ID(con1449076008755) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    CheckGoalKick::~CheckGoalKick()
    {
        /*PROTECTED REGION ID(dcon1449076008755) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void CheckGoalKick::run(void* msg)
    {
        /*PROTECTED REGION ID(run1449076008755) ENABLED START*/ //Add additional options here
        // get sensor data from WM and check validity
        ownPos = wm->rawSensorData.getOwnPositionVision();
        egoBallPos = wm->ball.getEgoBallPosition();

        std::cout << "OwnPos:     " << ownPos << std::endl;
        std::cout << "EgoBallPos: " << egoBallPos << std::endl;
        std::cout << "HaveBall: " << (wm->ball.haveBall() ? "true" : "false") << std::endl;

        if (ownPos == nullptr || egoBallPos == nullptr || !wm->ball.haveBall())
        {
            return;
        }

        shared_ptr < geometry::CNPoint2D > hitPoint = computeHitPoint(ownPos->x, ownPos->y, ownPos->theta);

        if (hitPoint && checkGoalKeeper(hitPoint) && checkShootPossibility(hitPoint))
        {
            kicking (hitPoint);
        }

        // console output
        cout << "==========================================================================" << endl;
        cout << "hits the goal: " << (hitPoint ? "true" : "false") << endl;
        if (hitPoint)
        {
            cout << "check goal keeper: " << (checkGoalKeeper(hitPoint) ? "true" : "false") << endl;
            cout << "check shoot possibility: " << (checkShootPossibility(hitPoint) ? "true" : "false") << endl;
            cout << "kick power: " << cout_kickpower << endl;
            cout << "kicking = " << cout_kicking << endl;
            cout << "minimum distance to obstacle: " << minOwnDistObs << endl;
        }

        /*PROTECTED REGION END*/
    }
    void CheckGoalKick::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1449076008755) ENABLED START*/ //Add additional options here
        cout << "Start run CheckGoalKick <=============================================================" << endl;
        field = msl::MSLFootballField::getInstance();
        auto rules = msl::Rules::getInstance();
        // space required to miss a robot (no offset because robots are pyramids)
        minOppYDist = rules->getBallRadius() + rules->getRobotRadius();

        readConfigParameters();

        // cout variables
        cout_kickpower = 0;
        cout_kicking = false;
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1449076008755) ENABLED START*/ //Add additional methods here
    void CheckGoalKick::readConfigParameters()
    {
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        minObsDistGoal = (*sc)["GoalKick"]->get<double>("GoalKick.Default.minObsDistGoal", NULL);
        minOwnDistGoal = (*sc)["GoalKick"]->get<double>("GoalKick.Default.minOwnDistGoal", NULL);
        minKickPower = (*sc)["GoalKick"]->get<double>("GoalKick.Default.minKickPower", NULL);
        keeperDistGoal = (*sc)["GoalKick"]->get<double>("GoalKick.Default.keeperDistGoal", NULL);

        // is necessary to calculate the minimum distance to obstacle so that it is possible to shoot a goal
        // close to goal --> distance to robot is higher
        closeGoalDist = (*sc)["GoalKick"]->get<double>("GoalKick.OwnDistObs.closeGoalDist", NULL);
        // far from goal --> distance to robot can be smaller
        farGoalDist = (*sc)["GoalKick"]->get<double>("GoalKick.OwnDistObs.farGoalDist", NULL);

    }

    /**
     * Calculates, whether a robot with the given position is hitting the goal.
     */
    shared_ptr<geometry::CNPoint2D> CheckGoalKick::computeHitPoint(double posX, double posY, double alloAngle)
    {
        double xDist2OppGoalline = this->field->FieldLength / 2 - posX;

        // normalize the position angle
        alloAngle = geometry::GeometryCalculator::normalizeAngle(alloAngle);

        // get the kicker angle from position angle (+/- M_PI)
        if (alloAngle < 0)
        {
            alloAngle += M_PI;
        }
        else
        {
            alloAngle -= M_PI;
        }

        if (alloAngle > M_PI / 2 || alloAngle < -M_PI / 2)
        {
            // you are aiming away from the opponent goal line
            return shared_ptr<geometry::CNPoint2D>();
        }

        double yHitGoalline = posY + xDist2OppGoalline * tan(alloAngle);
        // TODO reduce goalPost->y by (ball radius + safety margin)
        if (abs(yHitGoalline) < this->field->posLeftOppGoalPost()->y)
        {
            // you will hit the goal
            return make_shared < geometry::CNPoint2D > (this->field->FieldLength / 2, yHitGoalline);
        }

        return shared_ptr<geometry::CNPoint2D>();
    }

    /*
     * checks if there is an obstacle between robot and goal.
     *
     * @return true if it is possible to shoot at the enemy goal
     */
    bool CheckGoalKick::checkShootPossibility(shared_ptr<geometry::CNPoint2D> hitPoint)
    {
        auto obstacles = wm->obstacles.getAlloObstaclePoints();
        if (obstacles == nullptr || obstacles->size() == 0)
        {
            return true;
        }

        // new min distance to obstacle depends on distance to goal
        double distGoal = ownPos->distanceTo(hitPoint);

        // TODO magic number manipulation
        if (distGoal < farGoalDist)
        {
            minOwnDistObs = -0.2 * (distGoal / 1000 - minOwnDistGoal / 1000) + closeGoalDist;
        }
        else
        {
            minOwnDistObs = -0.2 * (distGoal / 1000 - minOwnDistGoal / 1000) + farGoalDist;
        }

        for (auto obs : *obstacles)
        {
            if (obs->x < ownPos->x)
            {
                continue; // obs is behind us
            }

            if (obs->distanceTo(ownPos) > minOwnDistObs)
            {
                continue; // obs is far away
            }

            if (abs(obs->alloToEgo(*ownPos)->y) < minOppYDist)
            {
                return false;
            }
        }

        return true;
    }

    void CheckGoalKick::kicking(shared_ptr<geometry::CNPoint2D> hitPoint)
    {
        msl_actuator_msgs::KickControl kc;
        double dist2HitPoint = ownPos->distanceTo(hitPoint);

        kc.enabled = true;
        if (dist2HitPoint > 5500)
        {
            cout_kickpower = (dist2HitPoint / 1000 - minOwnDistGoal / 1000) * 100 + minKickPower;
            kc.power = (dist2HitPoint / 1000 - minOwnDistGoal / 1000) * 100 + minKickPower;
        }
        else
        {
            cout_kickpower = minKickPower;
            kc.power = minKickPower;
        }
        cout_kicking = true;
        send(kc);
    }

    /**
     *
     * @return true if no opponent is on/near our hitPoint
     */
    bool CheckGoalKick::checkGoalKeeper(shared_ptr<geometry::CNPoint2D> hitPoint)
    {
        auto opps = wm->robots.opponents.getOpponentsAlloClustered();
        if (opps == nullptr || opps->size() == 0)
        {
            return true;
        }

        for (auto opp : *opps)
        {
            if (opp->distanceTo(hitPoint) < keeperDistGoal)
            {
                return false;
            }
        }
        return true;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
