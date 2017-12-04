#include "MSLWorldModel.h"
#include <WhiteBoard.h>

using supplementary::InformationElement;
using supplementary::InfoBuffer;
using supplementary::InfoTime;

namespace msl
{

WhiteBoard::WhiteBoard(MSLWorldModel *wm)
    : passMsgBuffer(10)
    , watchBallMsgBuffer(10)
{
    this->wm = wm;
}

WhiteBoard::~WhiteBoard()
{
}

void WhiteBoard::processPassMsg(msl_helper_msgs::PassMsgPtr msg)
{
    InfoTime time = wm->getTime();

    auto passMsg = make_shared<InformationElement<msl_helper_msgs::PassMsg>>(
        *msg, time, msg->validFor, 1.0); // TODO: correct validity time? certainty?
    passMsgBuffer.add(passMsg);
}

InfoBuffer<msl_helper_msgs::PassMsg> &WhiteBoard::getPassMsgBuffer()
{
    return this->passMsgBuffer;
}

void WhiteBoard::processWatchBallMsg(msl_helper_msgs::WatchBallMsgPtr msg)
{
    InfoTime time = wm->getTime();

    auto watchMsg = make_shared<InformationElement<msl_helper_msgs::WatchBallMsg>>(
        *msg, time, msg->validFor, 1.0); // TODO: correct validity time? certainty?
    watchBallMsgBuffer.add(watchMsg);
}

InfoBuffer<msl_helper_msgs::WatchBallMsg> &WhiteBoard::getWatchBallMsgBuffer()
{
    return this->watchBallMsgBuffer;
}

} /* namespace msl */
