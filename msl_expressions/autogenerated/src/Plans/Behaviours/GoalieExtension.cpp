using namespace std;
#include "Plans/Behaviours/GoalieExtension.h"

/*PROTECTED REGION ID(inccpp1459249216387) ENABLED START*/ //Add additional includes here
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1459249216387) ENABLED START*/ //initialise static variables here
    ExperimentalRingbuffer::ExperimentalRingbuffer(int size)
    {
        index = 0;
        indexMax = size;
    }

    shared_ptr<geometry::CNPoint2D> ExperimentalRingbuffer::getAvgPoint(int count)
    {
        if (count > indexMax)
        {
            count = indexMax;
        }
        if (count < 1)
        {
            count = 1;
        }
        int count2 = 0;
        double sum_gewichte = 0;
        shared_ptr < geometry::CNPoint2D > tmp = make_shared < geometry::CNPoint2D > (0, 0);
        for (int i = 0; count2 < count && i < indexMax; ++i)
        {
            int pos = index - i;
            if (pos < 0)
                pos += indexMax;

            try
            {
                if (buffer.at(pos) != nullptr)
                {
                    tmp = tmp + buffer.at(pos) * gewichte.at(pos);
                    ++count2;
                    sum_gewichte += gewichte.at(pos);
                }
            }
            catch (const std::out_of_range& e)
            {
            }

        }
        if (sum_gewichte == 0)
            return nullptr;
        tmp = tmp / sum_gewichte;
        return tmp;
    }

    void ExperimentalRingbuffer::addPoint(shared_ptr<geometry::CNPoint2D> p)
    {
        addPoint(p, 1);
    }

    void ExperimentalRingbuffer::overWrite(shared_ptr<geometry::CNPoint2D> p)
    {
        overWrite(p, 1);
    }

    void ExperimentalRingbuffer::addPoint(shared_ptr<geometry::CNPoint2D> p, double g)
    {
        if (++index > indexMax)
        {
            index = 0;
        }
        buffer.at(index) = p;
        if (g < 0.0001)
        {
            g = 0.0001;
        }
        gewichte.at(index) = g;
    }

    void ExperimentalRingbuffer::overWrite(shared_ptr<geometry::CNPoint2D> p, double g)
    {
        try
        {
            buffer.at(index) = p;
        }
        catch (const std::out_of_range& e)
        {
            buffer.push_back(p);
        }

        if (g < 0.0001)
        {
            g = 0.0001;
        }
        try
        {
            gewichte.at(index) = g;
        }
        catch (const std::out_of_range& e)
        {
            gewichte.push_back(g);
        }

    }

    /*PROTECTED REGION END*/
    GoalieExtension::GoalieExtension() :
            DomainBehaviour("GoalieExtension")
    {
        /*PROTECTED REGION ID(con1459249216387) ENABLED START*/ //Add additional options here
        useExt1 = (*this->sc)["Behaviour"]->get<bool>("Goalie.UseExt1", NULL);
        useExt2 = (*this->sc)["Behaviour"]->get<bool>("Goalie.UseExt2", NULL);
        useExt3 = (*this->sc)["Behaviour"]->get<bool>("Goalie.UseExt3", NULL);
        useKicker = (*this->sc)["Behaviour"]->get<bool>("Goalie.UseKicker", NULL);
        KICKER_WAIT_TIME = 40000000000;
        lastKickerTime = wm->getTime();
        ballGoalProjection = new ExperimentalRingbuffer(20);
        ballVelocity = new ExperimentalRingbuffer(10);
        ballInAirTimestamp = 0;

        /*PROTECTED REGION END*/
    }
    GoalieExtension::~GoalieExtension()
    {
        /*PROTECTED REGION ID(dcon1459249216387) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void GoalieExtension::run(void* msg)
    {
        /*PROTECTED REGION ID(run1459249216387) ENABLED START*/ //Add additional options here
        auto ownPos = wm->rawSensorData->getOwnPositionVision();
        if (ownPos == nullptr)
            return;

        auto ballPos = wm->ball->getEgoBallPosition();
        if (ballPos == nullptr)
            return;

        //kick
        msl_actuator_msgs::KickControl km;
        long currentTime = wm->getTime();
        if (currentTime - lastKickerTime >= KICKER_WAIT_TIME)
        {
            if (useKicker == true && ballPos != nullptr && ballPos->length() < 420
                    && (abs(ballPos->angleTo()) - M_PI) < 0.52)
            {

                km.enabled = true;
                km.kicker = 1;
                km.power = 100;
                send(km);
                lastKickerTime = wm->getTime();
            }
        }
        if (wm->rawSensorData->getLastMotionCommand() == nullptr)
            return;
        bm_last = msl_actuator_msgs::MotionControl();
        bm_last.motion.translation = wm->rawSensorData->getLastMotionCommand()->motion.translation;
        bm_last.motion.rotation = wm->rawSensorData->getLastMotionCommand()->motion.rotation;
        bm_last.motion.angle = wm->rawSensorData->getLastMotionCommand()->motion.angle;
        auto ballPosAllo = ballPos->egoToAllo(*ownPos);
        long now = wm->getTime() / 1000000;
        auto ballPos3D = wm->ball->getBallPoint3D();
        if (ballPos3D != nullptr)
        {
            if (ballPos3D->z > 500)
            {
                ballInAirTimestamp = now;
            }
        }

        auto ballV3D = wm->ball->getBallVel3D();
        if (ballV3D != nullptr && ballPos != nullptr)
        {

            auto ballV3DAllo = ballV3D->egoToAllo(*ownPos);
            double velo = sqrt(ballV3D->x * ballV3D->x + ballV3D->y * ballV3D->y + ballV3D->z * ballV3D->z);
            if (velo > 1000)
            {

                double diffAngle = abs(wm->rawSensorData->getLastMotionCommand()->motion.angle - bm_last.motion.angle);
                double diffTrans = abs(
                        wm->rawSensorData->getLastMotionCommand()->motion.translation - bm_last.motion.translation);
                double diffRot = abs(
                        wm->rawSensorData->getLastMotionCommand()->motion.rotation - bm_last.motion.rotation);
                double distBall = ballPos->length();

                double speed = wm->rawSensorData->getLastMotionCommand()->motion.translation;
                double g = 0;

                double g1 = 1.0 / (1.0 + exp(0.03 * (speed - 100.0))); //speed
                double g2 = 1.0 / (1.0 + exp(10 * (diffAngle - 0.5)));
                g2 = 1;
                double g3 = 1.0 / (1.0 + exp(0.03 * (diffTrans - 100)));
                g3 = 1;

                double g4 = 1.0 / (1.0 + exp(10 * (diffRot - 100)));
                double g5 = 1.0 / (1.0 + exp(0.0006 * (distBall - 6000)));
                g = g1 * g2 * g3 * g4 * g5;

                shared_ptr < geometry::CNPoint3D > ballVelo3DAllo = make_shared < geometry::CNPoint3D
                        > (ballV3DAllo->x, ballV3DAllo->y, ballV3DAllo->z);

                double x = -wm->field->getFieldLength() / 2 + 100;
                double timeBallToGoal = (x - ballPosAllo->x) / ballVelo3DAllo->x;
                //Console.WriteLine("ghgh timeBallToGoal " + timeBallToGoal);
                if (timeBallToGoal > 0)
                {
                    double y = ballPosAllo->y + ballVelo3DAllo->y * timeBallToGoal;
                    if (abs(y) < wm->field->getGoalWidth() / 2 + 2000)
                    {
                        if (y > wm->field->getGoalWidth() / 2)
                            y = wm->field->getGoalWidth() / 2;
                        if (y < -wm->field->getGoalWidth() / 2)
                            y = -wm->field->getGoalWidth() / 2;
                        auto dstPoint = make_shared < geometry::CNPoint2D > (x, y);
                        auto dstPointEgo = dstPoint->alloToEgo(*ownPos);

                        auto lookAt = make_shared < geometry::CNPoint2D > (0, 0);
                        auto lookAtEgo = lookAt->alloToEgo(*ownPos);
                        double lookAtAngle = lookAtEgo->angleTo() + M_PI;

                        ballVelo3DAllo = ballVelo3DAllo->normalize();

                        ballGoalProjection->overWrite(dstPoint, g);
                        ballVelocity->overWrite(make_shared < geometry::CNPoint2D > (ballV3DAllo->x, ballV3DAllo->y),
                                                g);

                        int count = (int)(ballPos->length() * ballPos->length()) / 1000000;
                        if (count > 3)
                        {
                            count = 3;
                        }
                        dstPoint = ballGoalProjection->getAvgPoint(count);
                        dstPointEgo = dstPoint->alloToEgo(*ownPos);

                        auto ballVeloBuf = ballVelocity->getAvgPoint((int)(ballPos->length() - 2000) / 2000);
                        double veloBuf = sqrt(ballVeloBuf->x * ballVeloBuf->x + ballVeloBuf->y * ballVeloBuf->y);
                        double timeBallToGoal2 = ballPosAllo->distanceTo(wm->field->posOwnGoalMid()) / veloBuf;
                        double timeToDstPoint = dstPointEgo->length() / 800;
                        if (timeToDstPoint > timeBallToGoal2 && ballPos->length() < 5000)
                        {
                            if (useExt3 == true && dstPointEgo->angleTo() < 0)
                            {
                                km.extension = msl_actuator_msgs::KickControl::LEFT_EXTENSION;
                            }
                            else if (useExt2 == true)
                            {
                                km.extension = msl_actuator_msgs::KickControl::RIGHT_EXTENSION;
                            }
                            if (useExt3 == true || useExt2 == true)
                            {
                                km.extTime = 1000;
                                send(km);
                            }
                        }
                        else if (useExt1 == true && timeBallToGoal2 < 1.0 && abs(dstPointEgo->y - ownPos->y) < 400
                                && ballInAirTimestamp + 3000 > now)
                        {
                            km.extension = msl_actuator_msgs::KickControl::UPPER_EXTENSION;
                            km.extTime = 1000;
                            send(km);
                        }

                    }
                }
            }
        }

        /*PROTECTED REGION END*/
    }
    void GoalieExtension::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1459249216387) ENABLED START*/ //Add additional options here
        bm_last = msl_actuator_msgs::MotionControl();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1459249216387) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
