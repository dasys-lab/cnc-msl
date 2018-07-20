/*
 * WhiteBoard.h
 *
 *  Created on: Aug 28, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_WHITEBOARD_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_WHITEBOARD_H_

#include "InformationElement.h"
#include "RingBuffer.h"
#include "container/CNPoint2D.h"
#include "container/CNVelocity2D.h"
#include "msl_helper_msgs/PassMsg.h"
#include "msl_helper_msgs/WatchBallMsg.h"
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

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_WHITEBOARD_H_ */
