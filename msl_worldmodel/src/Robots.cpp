#include "Robots.h"

#include "MSLWorldModel.h"

namespace msl
{

using std::map;
using std::shared_ptr;
using std::make_shared;

Robots::Robots(MSLWorldModel *wm, int ringBufferLength)
    : opponents(wm, ringBufferLength)
    , teammates(wm, ringBufferLength)
{
    this->wm = wm;
}

Robots::~Robots()
{
}

void Robots::processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfoPtr msg, InfoTime creationTime)
{
    // prepare info element
    auto shwmMsgPtr = shared_ptr<msl_sensor_msgs::SharedWorldInfo>(
        msg.get(), [msg](msl_sensor_msgs::SharedWorldInfo *) mutable { msg.reset(); });
    auto infoElement = make_shared<InformationElement<msl_sensor_msgs::SharedWorldInfo>>(
        shwmMsgPtr, creationTime, this->maxValidity, msg->odom.certainty);

    // find buffer to put element into
    auto shwmDataBufferIter = this->shwmDataBuffers.find(msg->senderID);
    if (shwmDataBufferIter == this->shwmDataBuffers.end())
    {
        auto buffer = std::make_shared<InfoBuffer<msl_sensor_msgs::SharedWorldInfo>>(wm->getRingBufferLength());
        this->shwmDataBuffers.emplace(msg->senderID, buffer);
        buffer->add(infoElement);
    }
    else
    {
        shwmDataBufferIter->second->add(infoElement);
    }

    // Forward position to teammates container
    this->teammates.integrateTeammatesPosition(msg, creationTime);
}

const InfoBuffer<msl_sensor_msgs::SharedWorldInfo> *Robots::getSHWMDataBuffer(int robotID) const
{
    auto shwmDataBufferIter = this->shwmDataBuffers.find(robotID);
    if (shwmDataBufferIter == this->shwmDataBuffers.end())
    {
        return nullptr;
    }
    else
    {
        return shwmDataBufferIter->second.get();
    }
}
} /* namespace msl */
