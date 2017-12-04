#pragma once

#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNVecAllo.h>
#include <supplementary/InformationElement.h>
#include <supplementary/InfoBuffer.h>

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

    supplementary::InfoBuffer<msl_helper_msgs::PassMsg>& getPassMsgBuffer();
    supplementary::InfoBuffer<msl_helper_msgs::WatchBallMsg>& getWatchBallMsgBuffer();

  private:
    MSLWorldModel *wm;
    supplementary::InfoBuffer<msl_helper_msgs::PassMsg> passMsgBuffer;
    supplementary::InfoBuffer<msl_helper_msgs::WatchBallMsg> watchBallMsgBuffer;
};

} /* namespace msl */
