/*
* DribbleCalibrationContainer.h
 *
 *  Created on: Jul 22, 2016
 *      Author: Carpe Noctem
 */

#pragma once

#include <msl_actuator_msgs/MotionControl.h>
#include <msl_robot/dribbleCalibration/behaviours/DribbleBackward.h>
#include <msl_robot/dribbleCalibration/behaviours/DribbleForward.h>
#include <msl_robot/dribbleCalibration/behaviours/DribbleRotateLeft.h>
#include <msl_robot/dribbleCalibration/behaviours/DribbleRotateRight.h>
#include <msl_robot/dribbleCalibration/container/DribbleCalibrationQuery.h>

#include <cnc_geometry/CNVecEgo.h>

#include <memory>
#include <vector>

//#define DEBUG_DC

namespace msl
{
class DribbleCalibrationContainer
{
  public:
    DribbleCalibrationContainer();
    virtual ~DribbleCalibrationContainer();

    enum Param
    {
        DribbleForwardParm,
        DribbleBackwardParm,
        DribbleLeftParm,
        DribbleRightParm,
        RotateLeftParm,
        RotateRightPram,
        ErrParm
    };

    enum MethodParam
    {
Move,
        AdaptParams,
        WriteConfigParam,
        ResetParams,
        SaveParams
    };

    std::shared_ptr<DribbleCalibrationQuery> paramToMove(Param param, int trans);
    void adaptParam(Param param);
    void writeConfigParameres(Param parm);
    void resetParameters(Param parm);
    void saveParameters(Param parm);

    // opticalFlow stuff
    double getAverageOpticalFlowXValue(const std::vector<geometry::CNVecEgo> &queue);
    double getAverageOpticalFlowYValue(const std::vector<geometry::CNVecEgo> &queue);

    bool queueFilled;
    bool fillOpticalFlowQueue(int queueSize, std::vector<geometry::CNVecEgo> &opQueue);

  private:
    msl::MSLWorldModel *wm;

    enum OPValue
    {
        XValue,
        YValue,
        QOSValue
    };

    std::shared_ptr<DribbleCalibrationQuery> callBehaviour(MethodParam mParm, Param parm, int trans = 0);
    std::shared_ptr<DribbleCalibrationQuery> callMethod(ICalibration *behavior, MethodParam parm, int trans = 0);

    // calibration behaviours
    DribbleForward df;
    DribbleBackward db;
    DribbleRotateLeft drl;
    DribbleRotateRight drr;

    double getAverageOpticalFlowValue(OPValue value, const std::vector<geometry::CNVecEgo> &queue);
    std::shared_ptr<msl_actuator_msgs::MotionControl> setNaN(std::shared_ptr<msl_actuator_msgs::MotionControl> mc);
};

} /* namespace alica */
