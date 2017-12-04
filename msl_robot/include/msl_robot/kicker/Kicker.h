#pragma once

#include "KickCurve.h"

#include <supplementary/InfoBuffer.h>
#include <supplementary/InformationElement.h>
#include <SystemConfig.h>
#include <msl_actuator_msgs/KickControl.h>
#include <nonstd/optional.hpp>

#include <memory>
namespace geometry
{
class CNPointEgo;
}
namespace msl
{

class MSLWorldModel;
class Kicker
{
  public:
    Kicker(MSLWorldModel *wm);
    virtual ~Kicker();
    bool init();
    float getKickerVoltage();
    void setKickerVoltage(float kickerVoltage);
    int getKickPowerPass(double dist);
    int getKickPowerSlowPass(double dist);
    int getKickerCount();
    int getKickPower(double dist, double height, double velo);
    bool mayShoot();
    nonstd::optional<geometry::CNPointEgo> getFreeGoalVector();
    double minFree(double angle, double width, const std::vector<double> &distanceScan) const;
    double getPassKickpower(double dist, double arrivalTime);
    double getPassVelocity(double kickpower);
    double getKickPowerForLobShot(double dist, double height, double heightTolerance = 30.0);
    double getPreciseShotMaxDistance();
    double getPreciseShotMaxTolerance();
    double getPreciseShotMinDistance();
    int getShortPassPower();
    bool lowShovelSelected;
    static double kickerAngle;

    void processKickConstrolMsg(msl_actuator_msgs::KickControl &km);

  private:
    MSLWorldModel *wm;
    float kickerVoltage;
    const supplementary::InfoTime maxValidity = 1000000000;

  protected:
    supplementary::SystemConfig *sc;
    double TWO_THIRD_PI;
    int kickerCount;
    double ballisticCurveOffset;
    double minVoltage;
    double maxVoltage;
    double powerMult;

    double preciseShotMaxDistance;
    double preciseShotMinDistance;
    double preciseShotMaxTolerance;
    int shortPassPower;

    double kickerAngleOffset;

    KickCurve *kick2GoalCurve;
    KickCurve *kickHighPass;
    KickCurve *kickLowPass;
    std::vector<geometry::CNPointEgo> validGoalPoints;

    supplementary::InfoBuffer<msl_actuator_msgs::KickControl> kickControlMsgs;
    int mod(int x, int y) const;
};

} /* namespace msl */

