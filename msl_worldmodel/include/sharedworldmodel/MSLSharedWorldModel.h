/*
 * MSLSharedWorldModel.h
 *
 *  Created on: 11.02.2015
 *      Author: tobi
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_SHAREDWORLDMODEL_MSLSHAREDWORLDMODEL_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_SHAREDWORLDMODEL_MSLSHAREDWORLDMODEL_H_

#include "SystemConfig.h"
#include <ros/ros.h>

namespace msl
{

class MSLWorldModel;
class MSLSharedWorldModel
{

  public:
    MSLSharedWorldModel(MSLWorldModel *wm);
    virtual ~MSLSharedWorldModel();
    void sendSharedWorldModelData(const ros::TimerEvent &e);

  private:
    int ownID;

    MSLWorldModel *wm;
    ros::Timer timer;
    ros::NodeHandle n;

    supplementary::SystemConfig *sc;
};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_SHAREDWORLDMODEL_MSLSHAREDWORLDMODEL_H_ */

