#pragma once

#include <ros/ros.h>

namespace supplementary
{
class SystemConfig;
}

namespace msl
{

class MSLWorldModel;
class Calibration
{
  public:
    Calibration(MSLWorldModel *wm);
    virtual ~Calibration();

    void sendKillMotionCommand();
    void sendStartMotionCommand();
    double getRobotRadius();
    void setRobotRadius(double newRadius);
    double adjustRobotRadius(double difference);

  private:
    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;
    ros::NodeHandle n;
    ros::Publisher processCommandPub;
};

} /* namespace msl */
