using namespace std;
#include "Plans/Behaviours/GoalieExtension.h"

/*PROTECTED REGION ID(inccpp1459249216387) ENABLED START*/ //Add additional includes here
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <cnc_geometry/CNPointAllo.h>
using std::make_shared;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1459249216387) ENABLED START*/ //initialise static variables here
    ExperimentalRingbuffer::ExperimentalRingbuffer(int size)
    {
        index = 0;
        indexMax = size;
    }

    ExperimentalRingbuffer::~ExperimentalRingbuffer()
    {
    }

    nonstd::optional<geometry::CNPointAllo> ExperimentalRingbuffer::getAvgPoint(int count)
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
        double sum_weights = 0;
        auto tmp = nonstd::make_optional<geometry::CNPointAllo>(0, 0);
        for (int i = 0; count2 < count && i < indexMax; ++i)
        {
            int pos = index - i;
            if (pos < 0)
                pos += indexMax;

            try
            {
                if (buffer.at(pos))
                {
                    tmp = nonstd::make_optional < geometry::CNPointAllo
                            > (tmp->x + buffer.at(pos)->x * weights.at(pos), tmp->y
                                    + buffer.at(pos)->y * weights.at(pos));
                    ++count2;
                    sum_weights += weights.at(pos);
                }
            }
            catch (const std::out_of_range& e)
            {
                std::cout << "GoalieExtension-Error: " << e.what() << std::endl;
            }
        }
        if (sum_weights == 0)
            return nonstd::nullopt;
        tmp = *tmp / sum_weights;
        return tmp;
    }

    void ExperimentalRingbuffer::addPoint(nonstd::optional<geometry::CNPointAllo> p)
    {
        addPoint(p, 1);
    }

    void ExperimentalRingbuffer::overWrite(nonstd::optional<geometry::CNPointAllo> p)
    {
        overWrite(p, 1);
    }

    void ExperimentalRingbuffer::addPoint(nonstd::optional<geometry::CNPointAllo> p, double w)
    {
        if (++index > indexMax)
        {
            index = 0;
        }
        buffer.at(index) = p;
        if (w < 0.0001)
        {
            w = 0.0001;
        }
        weights.at(index) = w;
    }

    void ExperimentalRingbuffer::overWrite(nonstd::optional<geometry::CNPointAllo> p, double w)
    {
        try
        {
            buffer.at(index) = p;
        }
        catch (const std::out_of_range& e)
        {
            buffer.push_back(p);
        }

        if (w < 0.0001)
        {
            w = 0.0001;
        }
        try
        {
            weights.at(index) = w;
        }
        catch (const std::out_of_range& e)
        {
            weights.push_back(w);
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
        delete this->ballGoalProjection;
        delete this->ballVelocity;
        /*PROTECTED REGION END*/
    }
    void GoalieExtension::run(void* msg)
    {
        /*PROTECTED REGION ID(run1459249216387) ENABLED START*/ //Add additional options here
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        if (!ownPos)
            return;

        auto ballPos = wm->ball->getPositionEgo();
        if (!ballPos)
            return;

        //kick
        msl_actuator_msgs::KickControl km;
        long currentTime = wm->getTime();
        if (currentTime - lastKickerTime >= KICKER_WAIT_TIME)
        {
            if (useKicker == true && ballPos && ballPos->length() < 420 && (abs(ballPos->angleZ()) - M_PI) < 0.52)
            {

                km.enabled = true;
                km.kicker = 1;
                km.power = 100;
                send(km);
                lastKickerTime = wm->getTime();
            }
        }
        auto lastMotionCmd = wm->rawSensorData->getLastMotionCommandBuffer().getLastValidContent();
        if (!lastMotionCmd)
            return;
        bm_last = msl_actuator_msgs::MotionControl();
        bm_last.motion.translation = lastMotionCmd->motion.translation;
        bm_last.motion.rotation = lastMotionCmd->motion.rotation;
        bm_last.motion.angle = lastMotionCmd->motion.angle;
        auto ballPosAllo = ballPos->toAllo(*ownPos);
        long now = wm->getTime() / 1000000;
        auto ballPos3D = wm->ball->getPositionAllo();
        if (ballPos3D)
        {
            if (ballPos3D->z > 500)
            {
                ballInAirTimestamp = now;
            }
        }

        auto ballV3D = wm->ball->getVelocityAllo();
        if (ballV3D && ballPos)
        {
            double velo = sqrt(ballV3D->x * ballV3D->x + ballV3D->y * ballV3D->y + ballV3D->z * ballV3D->z);
            if (velo > 1000)
            {

                auto motion = wm->rawSensorData->getLastMotionCommandBuffer().getLastValidContent();
                double diffAngle = abs(motion->motion.angle - bm_last.motion.angle);
                double diffTrans = abs(motion->motion.translation - bm_last.motion.translation);
                double diffRot = abs(motion->motion.rotation - bm_last.motion.rotation);
                double distBall = ballPos->length();

                double speed = motion->motion.translation;
                double g = 0;

                double g1 = 1.0 / (1.0 + exp(0.03 * (speed - 100.0))); //speed
                double g2 = 1.0 / (1.0 + exp(10 * (diffAngle - 0.5)));
                g2 = 1;
                double g3 = 1.0 / (1.0 + exp(0.03 * (diffTrans - 100)));
                g3 = 1;

                double g4 = 1.0 / (1.0 + exp(10 * (diffRot - 100)));
                double g5 = 1.0 / (1.0 + exp(0.0006 * (distBall - 6000)));
                g = g1 * g2 * g3 * g4 * g5;

                geometry::CNPointAllo ballVelo3DAllo = geometry::CNPointAllo(ballV3D->x, ballV3D->y, ballV3D->z);

                double x = -wm->field->getFieldLength() / 2 + 100;
                double timeBallToGoal = (x - ballPosAllo.x) / ballVelo3DAllo.x;
                //Console.WriteLine("ghgh timeBallToGoal " + timeBallToGoal);
                if (timeBallToGoal > 0)
                {
                    double y = ballPosAllo.y + ballVelo3DAllo.y * timeBallToGoal;
                    if (abs(y) < wm->field->getGoalWidth() / 2 + 2000)
                    {
                        if (y > wm->field->getGoalWidth() / 2)
                            y = wm->field->getGoalWidth() / 2;
                        if (y < -wm->field->getGoalWidth() / 2)
                            y = -wm->field->getGoalWidth() / 2;
                        auto dstPoint = nonstd::make_optional<geometry::CNPointAllo>(x, y);
                        auto dstPointEgo = dstPoint->toEgo(*ownPos);

                        auto lookAt = geometry::CNPointAllo(0, 0);
                        auto lookAtEgo = lookAt.toEgo(*ownPos);
                        double lookAtAngle = lookAtEgo.angleZ() + M_PI;

                        ballVelo3DAllo = ballVelo3DAllo.normalize();

                        ballGoalProjection->overWrite(*dstPoint, g);
                        auto velCoords = geometry::CNPointAllo(ballV3D->x, ballV3D->y, ballV3D->z);
                        ballVelocity->overWrite(velCoords, g);

                        int count = (int)(ballPos->length() * ballPos->length()) / 1000000;
                        if (count > 3)
                        {
                            count = 3;
                        }
                        dstPoint = ballGoalProjection->getAvgPoint(count);
                        dstPointEgo = dstPoint->toEgo(*ownPos);

                        auto ballVeloBuf = (ballVelocity->getAvgPoint((int)(ballPos->length() - 2000) / 2000));
                        double veloBuf = sqrt(ballVeloBuf->x * ballVeloBuf->x + ballVeloBuf->y * ballVeloBuf->y);
                        double timeBallToGoal2 = ballPosAllo.distanceTo(wm->field->posOwnGoalMid()) / veloBuf;
                        double timeToDstPoint = dstPointEgo.length() / 800;
                        if (timeToDstPoint > timeBallToGoal2 && ballPos->length() < 5000)
                        {
                            if (useExt3 == true && dstPointEgo.angleZ() < 0)
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
                        else if (useExt1 == true && timeBallToGoal2 < 1.0 && abs(dstPointEgo.y - ownPos->y) < 400
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
