#include "Robots.h"

#include "MSLWorldModel.h"
#include <engine/AlicaEngine.h>

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
	const supplementary::IAgentID* senderID = this->wm->getEngine()->getIDFromBytes(data->senderID.id);

    if (sharedWolrdModelData.find(senderID) == sharedWolrdModelData.end())
    {
        shared_ptr<RingBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>> buffer =
            make_shared<RingBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>>(wm->getRingBufferLength());

        pair<const supplementary::IAgentID*, shared_ptr<RingBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>>> pair(senderID, buffer);
        sharedWolrdModelData.insert(pair);
    }
    if (teammates.robotPositions.find(senderID) == teammates.robotPositions.end())
    {
        shared_ptr<RingBuffer<InformationElement<geometry::CNPosition>>> buffer =
            make_shared<RingBuffer<InformationElement<geometry::CNPosition>>>(wm->getRingBufferLength());
        pair<const supplementary::IAgentID*, shared_ptr<RingBuffer<InformationElement<geometry::CNPosition>>>> pair(senderID, buffer);
        teammates.robotPositions.insert(pair);
    }
    shared_ptr<InformationElement<geometry::CNPosition>> info = make_shared<InformationElement<geometry::CNPosition>>(
        make_shared<geometry::CNPosition>(data->odom.position.x, data->odom.position.y, data->odom.position.angle), wm->getTime());
    teammates.robotPositions.at(senderID)->add(info);

    shared_ptr<msl_sensor_msgs::SharedWorldInfo> shptr = make_shared<msl_sensor_msgs::SharedWorldInfo>();
    *shptr = *data;
    shared_ptr<InformationElement<msl_sensor_msgs::SharedWorldInfo>> infosh =
        make_shared<InformationElement<msl_sensor_msgs::SharedWorldInfo>>(shptr, wm->getTime());
    sharedWolrdModelData.at(senderID)->add(infosh);

}

shared_ptr<msl_sensor_msgs::SharedWorldInfo> Robots::getSHWMData(const supplementary::IAgentID* robotID, int index)
{
    auto shwm = sharedWolrdModelData.find(robotID);
    if (shwm != sharedWolrdModelData.end())
    {
        auto x = shwm->second->getLast(index);
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
