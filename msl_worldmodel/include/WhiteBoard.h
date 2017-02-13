#pragma once

#include "InformationElement.h"
#include "RingBuffer.h"

#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNVecAllo.h>

#include <msl_helper_msgs/PassMsg.h>
#include <msl_helper_msgs/WatchBallMsg.h>

#include <memory>

namespace msl
{

class MSLWorldModel;
class WhiteBoard
{
  public:
    WhiteBoard(MSLWorldModel *wm);
    virtual ~WhiteBoard();
    void processPassMsg(msl_helper_msgs::PassMsgPtr msg);
    void processWatchBallMsg(msl_helper_msgs::WatchBallMsgPtr msg);
    shared_ptr<msl_helper_msgs::PassMsg> getPassMsg(int index = 0);
    shared_ptr<msl_helper_msgs::WatchBallMsg> getWatchBallMsg(int index = 0);

  private:
    MSLWorldModel *wm;
    unsigned long maxInformationAge = 1000000000;
    RingBuffer<InformationElement<msl_helper_msgs::PassMsg>> passMsgs;
    RingBuffer<InformationElement<msl_helper_msgs::WatchBallMsg>> watchBallMsgs;
};

} /* namespace msl */
