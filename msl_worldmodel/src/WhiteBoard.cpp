/*
 * WhiteBoard.cpp
 *
 *  Created on: Aug 28, 2015
 *      Author: Stefan Jakob
 */

#include "MSLWorldModel.h"
#include <WhiteBoard.h>

namespace msl
{

WhiteBoard::WhiteBoard(MSLWorldModel *wm)
    : passMsgs(10)
    , watchBallMsgs(10)
{
    this->wm = wm;
}

WhiteBoard::~WhiteBoard()
{
}

void WhiteBoard::processPassMsg(msl_helper_msgs::PassMsgPtr msg)
{
    InfoTime time = wm->getTime();

    shared_ptr<msl_helper_msgs::PassMsg> p = make_shared<msl_helper_msgs::PassMsg>(*msg);
    shared_ptr<InformationElement<msl_helper_msgs::PassMsg>> passMsg = make_shared<InformationElement<msl_helper_msgs::PassMsg>>(p, time);
    passMsgs.add(passMsg);
}

shared_ptr<msl_helper_msgs::PassMsg> WhiteBoard::getPassMsg(int index)
{
    auto x = passMsgs.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > x->getInformation()->validFor)
    {
        return nullptr;
    }
    return x->getInformation();
}

void WhiteBoard::processWatchBallMsg(msl_helper_msgs::WatchBallMsgPtr msg)
{
    InfoTime time = wm->getTime();

    shared_ptr<msl_helper_msgs::WatchBallMsg> p = make_shared<msl_helper_msgs::WatchBallMsg>(*msg);
    shared_ptr<InformationElement<msl_helper_msgs::WatchBallMsg>> watchMsg = make_shared<InformationElement<msl_helper_msgs::WatchBallMsg>>(p, time);
    watchBallMsgs.add(watchMsg);
}

shared_ptr<msl_helper_msgs::WatchBallMsg> WhiteBoard::getWatchBallMsg(int index)
{
    auto x = watchBallMsgs.getLast(index);
    if (x == nullptr || wm->getTime() - x->timeStamp > x->getInformation()->validFor)
    {
        return nullptr;
    }
    return x->getInformation();
}

} /* namespace msl */
