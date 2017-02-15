/*
 * Robots.cpp
 *
 *  Created on: Feb 23, 2015
 *      Author: Stefan Jakob
 */

#include "MSLWorldModel.h"
#include <Robots.h>

namespace msl
{

Robots::Robots(MSLWorldModel *wm, int ringBufferLength)
    : opponents(wm, ringBufferLength)
    , teammates(wm, ringBufferLength)
{
    this->wm = wm;
    this->sc = supplementary::SystemConfig::getInstance();
    maxInformationAge = 1000000000;
}

Robots::~Robots()
{
    // TODO Auto-generated destructor stub
}

void Robots::processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfoPtr data)
{
    if (sharedWorldModelData.find(data->senderID) == sharedWorldModelData.end())
    {
        shared_ptr<InfoBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>> buffer =
            make_shared<InfoBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>>(wm->getRingBufferLength());
        pair<int, shared_ptr<InfoBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>>> pair(data->senderID, buffer);
        sharedWorldModelData.insert(pair);
    }
    if (teammates.robotPositions.find(data->senderID) == teammates.robotPositions.end())
    {
        shared_ptr<InfoBuffer<InformationElement<geometry::CNPositionAllo>>> buffer =
            make_shared<InfoBuffer<InformationElement<geometry::CNPositionAllo>>>(wm->getRingBufferLength());
        pair<int, shared_ptr<InfoBuffer<InformationElement<geometry::CNPositionAllo>>>> pair(data->senderID, buffer);
        teammates.robotPositions.insert(pair);
    }
    shared_ptr<InformationElement<geometry::CNPositionAllo>> info = make_shared<InformationElement<geometry::CNPositionAllo>>(
        make_shared<geometry::CNPositionAllo>(data->odom.position.x, data->odom.position.y, data->odom.position.angle), wm->getTime());
    teammates.robotPositions.at(data->senderID)->add(info);

    shared_ptr<msl_sensor_msgs::SharedWorldInfo> shptr = make_shared<msl_sensor_msgs::SharedWorldInfo>();
    *shptr = *data;
    shared_ptr<InformationElement<msl_sensor_msgs::SharedWorldInfo>> infosh =
        make_shared<InformationElement<msl_sensor_msgs::SharedWorldInfo>>(shptr, wm->getTime());
    sharedWorldModelData.at(data->senderID)->add(infosh);
}

shared_ptr<msl_sensor_msgs::SharedWorldInfo> Robots::getSHWMData(int robotID, int index)
{
    auto shwm = sharedWorldModelData.at(robotID);
    if (shwm != nullptr)
    {
        auto x = shwm->getLast(index);
        if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
        {
            return nullptr;
        }
        return x->getInformation();
    }
    return nullptr;
}
}

/* namespace alica */
