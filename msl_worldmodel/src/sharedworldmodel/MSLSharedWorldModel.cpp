/*
 * MSLSharedWorldModel.cpp
 *
 *  Created on: 11.02.2015
 *      Author: tobi
 */

#include "sharedworldmodel/MSLSharedWorldModel.h"
#include "MSLWorldModel.h"
#include <ios>

using namespace std;

namespace msl
{

MSLSharedWorldModel::MSLSharedWorldModel(MSLWorldModel *wm)
{

    this->wm = wm;
    timer = n.createTimer(ros::Duration(0.1), &MSLSharedWorldModel::sendSharedWorldModelData, this);

    sc = supplementary::SystemConfig::getInstance();
    ownID = sc->getOwnRobotID();
}

MSLSharedWorldModel::~MSLSharedWorldModel()
{
    // TODO Auto-generated destructor stub
}

void MSLSharedWorldModel::sendSharedWorldModelData(const ros::TimerEvent &e)
{
    wm->sendSharedWorldModelData();
}

} /* namespace msl */
