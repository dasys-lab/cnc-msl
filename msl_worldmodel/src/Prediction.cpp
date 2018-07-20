/*
 * Prediction.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: sni
 */

#include "Prediction.h"

#include <array>
#include <mutex>
#include <vector>

#include "GeometryCalculator.h"
#include "MSLWorldModel.h"
#include "RawSensorData.h"
#include "SystemConfig.h"
#include <msl_actuator_msgs/MotionControl.h>

namespace msl
{

Prediction::Prediction()
{
    auto sc = supplementary::SystemConfig::getInstance();
    this->maxRotationAccel = (*sc)["Motion"]->tryGet<double>(15.0, "Motion.MotionControl.MaxRotation", NULL);
    this->magicNumber = 1000000000.0;
    debugAngle.open("/tmp/angle.txt");
    debugAnglePosition.open("/tmp/anglePos.txt");
    debugRotationVel.open("/tmp/rotationVel.txt");
}

Prediction::~Prediction()
{
    // nothing to do here
}

void Prediction::monitoring()
{
    auto wm = MSLWorldModel::get();
    auto ownPos = wm->rawSensorData->getOwnPositionVision();
    auto odo = wm->rawSensorData->getCorrectedOdometryInfo();

    if (ownPos && odo)
    {
        // plot '/tmp/angle.txt' u 1:2 w l, '/tmp/angle.txt' u 1:3 w l, '/tmp/angle.txt' u ($1 + 150000000):3 w l
        double angle = wm->prediction->angle(150);

        double alloAngle = ownPos->theta;
        if (alloAngle > M_PI)
            alloAngle -= 2 * M_PI;
        else if (alloAngle < -M_PI)
            alloAngle += 2 * M_PI;

        debugAngle << wm->getTime() << " " << alloAngle << " " << angle << std::endl << std::flush;

        //  plot '/tmp/anglePos.txt' u 1:2 w l, '/tmp/anglePos.txt' u 1:6 w l, '/tmp/anglePos.txt' u ($1 + 150000000):6 w l
        //  plot '/tmp/anglePos.txt' u 1:3 w l, '/tmp/anglePos.txt' u 1:7 w l, '/tmp/anglePos.txt' u ($1 + 150000000):7 w l
        auto anglePos = wm->prediction->angleAndPosition(150);

        if (anglePos)
        {
            double alloAngle = ownPos->theta;
            if (alloAngle > M_PI)
                alloAngle -= 2 * M_PI;
            else if (alloAngle < -M_PI)
                alloAngle += 2 * M_PI;

            debugAnglePosition << wm->getTime() << " " << alloAngle << " " << ownPos->x << " " << ownPos->y << " " << odo->motion.translation;
            debugAnglePosition << " " << anglePos->first->theta << " " << anglePos->first->x << " " << anglePos->first->y << " " << anglePos->second
                               << std::endl
                               << std::flush;
        }
    }
}

double Prediction::rotationVelocity(int ms)
{
    auto wm = MSLWorldModel::get();

    auto odo = wm->rawSensorData->getCorrectedOdometryInfo();
    if (false == odo)
        return 0;

    double ret = 0;
    if (wm->rawSensorData->getLastMotionCommand() == 0)
        return odo->motion.rotation;

    long targetTimeMs = wm->getTime() - ms * 1000000; // Converting milli seconds to nano seconds
    int i = 0;
    auto motionCommand = wm->rawSensorData->getLastMotionCommand(i);
    std::vector<std::shared_ptr<msl_actuator_msgs::MotionControl>> cmd;

    {
        // TODO lock
        //    std::lock_guard<std::mutex>(wm->)
        while (motionCommand && motionCommand->timestamp > targetTimeMs)
        {
            cmd.push_back(motionCommand);
            motionCommand = wm->rawSensorData->getLastMotionCommand(i);
            i++;
        }
    }

    ret = odo->motion.rotation;
    long delta;
    double accel;

    for (int i = cmd.size() - 1; i >= 0; --i)
    {
        auto mc = cmd[i];
        if (false == mc)
            continue;
        delta = mc->timestamp - targetTimeMs;
        if (delta < 0)
            continue;
        accel = (mc->motion.rotation - ret) / delta;
        accel *= this->magicNumber;
        if (accel < 0)
            accel = -1 * min(this->maxRotationAccel, abs(accel));
        else
            accel = min(this->maxRotationAccel, abs(accel));
        ret += accel * delta / this->magicNumber;

        targetTimeMs += delta;
    }

    return ret;
}

double Prediction::angle(int ms)
{
    auto wm = MSLWorldModel::get();

    auto odo = wm->rawSensorData->getCorrectedOdometryInfo();
    if (false == odo)
        return 0;

    double velo = 0;
    double angle = 0;

    if (false == wm->rawSensorData->getLastMotionCommand())
        return odo->position.angle;

    long targetTimeMs = wm->getTime() - ms * 1000000; // Converting milli seconds to nano seconds
    int i = 0;
    auto motionCommand = wm->rawSensorData->getLastMotionCommand(i);
    std::vector<std::shared_ptr<msl_actuator_msgs::MotionControl>> cmd;

    {
        // TODO lock
        //    std::lock_guard<std::mutex>(wm->)

        while (motionCommand && motionCommand->timestamp > targetTimeMs)
        {
            cmd.push_back(motionCommand);
            motionCommand = wm->rawSensorData->getLastMotionCommand(i);
            i++;
        }
    }

    velo = odo->motion.rotation;
    angle = odo->position.angle;
    long delta;
    double accel;

    for (int i = cmd.size() - 1; i >= 0; --i)
    {
        auto mc = cmd[i];
        if (false == mc)
            continue;
        delta = mc->timestamp - targetTimeMs;
        if (delta < 0)
            continue;

        accel = (mc->motion.rotation - velo) / delta;
        accel *= this->magicNumber;
        if (accel < 0)
            accel = -1 * min(this->maxRotationAccel, abs(accel));
        else
            accel = min(this->maxRotationAccel, abs(accel));
        angle += velo * (delta / this->magicNumber) + 0.5 * accel * (delta / this->magicNumber) * (delta / this->magicNumber);
        velo += accel * delta / this->magicNumber;

        targetTimeMs += delta;
    }

    return geometry::normalizeAngle(angle);
}

std::unique_ptr<std::pair<shared_ptr<geometry::CNPosition>, double>> Prediction::angleAndPosition(int ms)
{
    auto wm = MSLWorldModel::get();

    auto odo = wm->rawSensorData->getCorrectedOdometryInfo();
    if (false == odo)
        return nullptr;

    double velo = 0;
    double angle = 0;
    double x, y, trans, transX, transY;

    if (wm->rawSensorData->getLastMotionCommand() == 0)
        return 0;

    long targetTimeMs = wm->getTime() - ms * 1000000; // Converting milli seconds to nano seconds
    int i = 0;
    auto motionCommand = wm->rawSensorData->getLastMotionCommand(i);
    std::vector<std::shared_ptr<msl_actuator_msgs::MotionControl>> cmd;

    {
        // TODO lock
        //    std::lock_guard<std::mutex>(wm->)

        while (motionCommand && motionCommand->timestamp > targetTimeMs)
        {
            cmd.push_back(motionCommand);
            motionCommand = wm->rawSensorData->getLastMotionCommand(i);
            i++;
        }
    }

    velo = odo->motion.rotation;
    angle = odo->position.angle;
    double motionAngle = odo->motion.angle;
    x = odo->position.x; // Al
    y = odo->position.y; // Al
    trans = odo->motion.translation;
    transX = cos(motionAngle) * trans;
    transY = sin(motionAngle) * trans;

    long delta;
    double accel;
    std::shared_ptr<geometry::CNPoint2D> d = std::make_shared<geometry::CNPoint2D>();

    for (int i = cmd.size() - 1; i >= 0; --i)
    {
        auto mc = cmd[i];
        if (false == mc)
            continue;
        delta = mc->timestamp - targetTimeMs;
        if (delta < 0)
            continue;

        double rot = mc->motion.rotation;
        double ang = mc->motion.angle;
        double tra = mc->motion.translation;
        double deltaS = ((double)delta) / this->magicNumber;

        // Angle by Taker (Hendrik)
        accel = (rot - velo) / deltaS;
        if (std::isnan(accel))
            continue;

        if (accel < 0)
            accel = -1 * min(this->maxRotationAccel, abs(accel));
        else
            accel = min(this->maxRotationAccel, abs(accel));

        angle += velo * deltaS + 0.5 * accel * deltaS * deltaS;
        velo += accel * deltaS;

        // Position by Taker
        d->x = (cos(ang) * tra - transX) / deltaS;
        d->y = (sin(ang) * tra - transY) / deltaS;

        if (d->length() > MAX_ACCELERATION)
        {
            d = d->normalize() * MAX_ACCELERATION;
        }

        d->x = d->x + transX;
        d->y = d->y + transY;

        trans = d->length();
        transX = d->x;
        transY = d->y;

        double a = atan2(transY, transX);

        d->x = (cos(a) * trans) * deltaS;
        d->y = (sin(a) * trans) * deltaS;
        d->rotate(angle);

        x += d->x;
        y += d->y;

        targetTimeMs += delta;
    }

    angle = geometry::normalizeAngle(angle);

    auto pos = make_shared<geometry::CNPosition>(x, y, angle);
    std::unique_ptr<std::pair<shared_ptr<geometry::CNPosition>, double>> returnVal(new std::pair<shared_ptr<geometry::CNPosition>, double>(pos, trans));

    return std::move(returnVal);
}
}
/* namespace ice */
