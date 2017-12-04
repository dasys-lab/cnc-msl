#include "Prediction.h"

#include "MSLWorldModel.h"
#include "RawSensorData.h"

#include <SystemConfig.h>
#include <cnc_geometry/Calculator.h>
#include <msl_actuator_msgs/MotionControl.h>

#include <array>
#include <mutex>
#include <vector>

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
}

void Prediction::monitoring()
{
    auto wm = MSLWorldModel::get();
    auto ownPosInfo = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValid();
    auto odoInfo = wm->rawSensorData->getCorrectedOdometryBuffer().getLastValid();

    if (ownPosInfo && odoInfo)
    {
        auto ownPos = ownPosInfo->getInformation();
        auto odo = odoInfo->getInformation();

        // plot '/tmp/angle.txt' u 1:2 w l, '/tmp/angle.txt' u 1:3 w l, '/tmp/angle.txt' u ($1 + 150000000):3 w l
        double angle = wm->prediction->angle(150);

        double alloAngle = ownPos.theta;
        if (alloAngle > M_PI)
            alloAngle -= 2 * M_PI;
        else if (alloAngle < -M_PI)
            alloAngle += 2 * M_PI;

        debugAngle << wm->getTime() << " " << alloAngle << " " << angle << std::endl << std::flush;

        //  plot '/tmp/anglePos.txt' u 1:2 w l, '/tmp/anglePos.txt' u 1:6 w l, '/tmp/anglePos.txt' u ($1 + 150000000):6
        //  w l
        //  plot '/tmp/anglePos.txt' u 1:3 w l, '/tmp/anglePos.txt' u 1:7 w l, '/tmp/anglePos.txt' u ($1 + 150000000):7
        //  w l
        auto anglePos = wm->prediction->angleAndPosition(150);

        if (anglePos)
        {
            double alloAngle = ownPos.theta;
            if (alloAngle > M_PI)
                alloAngle -= 2 * M_PI;
            else if (alloAngle < -M_PI)
                alloAngle += 2 * M_PI;

            debugAnglePosition << wm->getTime() << " " << alloAngle << " " << ownPos.x << " " << ownPos.y << " "
                               << odo.motion.translation;
            debugAnglePosition << " " << anglePos->first->theta << " " << anglePos->first->x << " "
                               << anglePos->first->y << " " << anglePos->second << std::endl
                               << std::flush;
        }
    }
}

double Prediction::rotationVelocity(int ms)
{
    auto wm = MSLWorldModel::get();

    auto odoInfo = wm->rawSensorData->getCorrectedOdometryBuffer().getLastValid();
    if (!odoInfo)
        return 0;

    auto odo = odoInfo->getInformation();

    double ret = 0;

    auto &motionCommandBuffer = wm->rawSensorData->getLastMotionCommandBuffer();

    if (motionCommandBuffer.getLast())
        return odo.motion.rotation;

    long targetTimeMs = wm->getTime() - ms * 1000000; // Converting milli seconds to nano seconds
    int i = 0;
    auto motionCommand = motionCommandBuffer.getLast(i);
    std::vector<msl_actuator_msgs::MotionControl> cmds;

    {
        // TODO lock
        //    std::lock_guard<std::mutex>(wm->)
        while (motionCommand && motionCommand->getInformation().timestamp > targetTimeMs)
        {
            cmds.push_back(motionCommand->getInformation());
            i++;

            motionCommand = motionCommandBuffer.getLast(i);
            // TODO: i++ was here
        }
    }

    ret = odo.motion.rotation;
    long delta;
    double accel;

    for (int i = cmds.size() - 1; i >= 0; --i)
    {
        auto mc = cmds[i];

        delta = mc.timestamp - targetTimeMs;
        if (delta < 0)
            continue;
        accel = (mc.motion.rotation - ret) / delta;
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

    auto odoInfo = wm->rawSensorData->getCorrectedOdometryBuffer().getLastValid();
    if (!odoInfo)
    {
        return 0;
    }

    auto odo = odoInfo->getInformation();

    double vel = 0;
    double angle = 0;

    auto &motionCommandBuffer = wm->rawSensorData->getLastMotionCommandBuffer();

    if (!motionCommandBuffer.getLast())
        return odo.position.angle;

    long targetTimeMs = wm->getTime() - ms * 1000000; // Converting milli seconds to nano seconds
    int i = 0;
    auto motionCommand = motionCommandBuffer.getLast(i);
    std::vector<msl_actuator_msgs::MotionControl> cmds;

    {
        // TODO lock
        //    std::lock_guard<std::mutex>(wm->)

        while (motionCommand && motionCommand->getInformation().timestamp > targetTimeMs)
        {
            cmds.push_back(motionCommand->getInformation());
            i++;

            motionCommand = motionCommandBuffer.getLast(i);
            // TODO: i++ was here
        }
    }

    vel = odo.motion.rotation;
    angle = odo.position.angle;
    long delta;
    double accel;

    for (int i = cmds.size() - 1; i >= 0; --i)
    {
        auto mc = cmds[i];

        delta = mc.timestamp - targetTimeMs;
        if (delta < 0)
            continue;

        accel = (mc.motion.rotation - vel) / delta;
        accel *= this->magicNumber;
        if (accel < 0)
            accel = -1 * min(this->maxRotationAccel, abs(accel));
        else
            accel = min(this->maxRotationAccel, abs(accel));
        angle +=
            vel * (delta / this->magicNumber) + 0.5 * accel * (delta / this->magicNumber) * (delta / this->magicNumber);
        vel += accel * delta / this->magicNumber;

        targetTimeMs += delta;
    }

    return geometry::normalizeAngle(angle);
}

std::unique_ptr<std::pair<shared_ptr<geometry::CNPositionAllo>, double>> Prediction::angleAndPosition(int ms)
{
    auto wm = MSLWorldModel::get();

    auto odoInfo = wm->rawSensorData->getCorrectedOdometryBuffer().getLastValid();
    if (!odoInfo)
    {
        return nullptr;
    }
    auto odo = odoInfo->getInformation();

    double vel = 0;
    double angle = 0;
    double x, y, trans, transX, transY;

    auto &motionCommandBuffer = wm->rawSensorData->getLastMotionCommandBuffer();

    if (!motionCommandBuffer.getLast())
        return nullptr;

    long targetTimeMs = wm->getTime() - ms * 1000000; // Converting milli seconds to nano seconds
    int i = 0;
    auto motionCommand = motionCommandBuffer.getLast(i);
    std::vector<msl_actuator_msgs::MotionControl> cmds;

    {
        // TODO lock
        //    std::lock_guard<std::mutex>(wm->)

        while (motionCommand && motionCommand->getInformation().timestamp > targetTimeMs)
        {
            cmds.push_back(motionCommand->getInformation());
            i++;

            motionCommand = motionCommandBuffer.getLast(i);
            // TODO: i++ was here
        }
    }

    vel = odo.motion.rotation;
    angle = odo.position.angle;
    double motionAngle = odo.motion.angle;
    x = odo.position.x; // Al
    y = odo.position.y; // Al
    trans = odo.motion.translation;
    transX = cos(motionAngle) * trans;
    transY = sin(motionAngle) * trans;

    long delta;
    double accel;
    geometry::CNPointAllo d;

    for (int i = cmds.size() - 1; i >= 0; --i)
    {
        auto mc = cmds[i];

        delta = mc.timestamp - targetTimeMs;
        if (delta < 0)
            continue;

        double rot = mc.motion.rotation;
        double ang = mc.motion.angle;
        double tra = mc.motion.translation;
        double deltaS = ((double)delta) / this->magicNumber;

        // Angle by Taker (Hendrik)
        accel = (rot - vel) / deltaS;
        if (std::isnan(accel))
            continue;

        if (accel < 0)
            accel = -1 * min(this->maxRotationAccel, abs(accel));
        else
            accel = min(this->maxRotationAccel, abs(accel));

        angle += vel * deltaS + 0.5 * accel * deltaS * deltaS;
        vel += accel * deltaS;

        // Position by Taker
        d.x = (cos(ang) * tra - transX) / deltaS;
        d.y = (sin(ang) * tra - transY) / deltaS;

        if (d.length() > MAX_ACCELERATION)
        {
            d = d.normalize() * MAX_ACCELERATION;
        }

        d.x = d.x + transX;
        d.y = d.y + transY;

        trans = d.length();
        transX = d.x;
        transY = d.y;

        double a = atan2(transY, transX);

        d.x = (cos(a) * trans) * deltaS;
        d.y = (sin(a) * trans) * deltaS;
        d = d.rotateZ(angle);

        x += d.x;
        y += d.y;

        targetTimeMs += delta;
    }

    angle = geometry::normalizeAngle(angle);

    auto pos = make_shared<geometry::CNPositionAllo>(x, y, angle);
    std::unique_ptr<std::pair<shared_ptr<geometry::CNPositionAllo>, double>> returnVal(
        new std::pair<shared_ptr<geometry::CNPositionAllo>, double>(pos, trans));

    return std::move(returnVal);
}

} /* namespace ice */
