/*
 * WhiteBoard.h
 *
 *  Created on: Aug 28, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_WHITEBOARD_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_WHITEBOARD_H_

#include "container/CNPoint2D.h"
#include "container/CNVelocity2D.h"
#include "RingBuffer.h"
#include "InformationElement.h"
#include <memory>
#include "msl_helper_msgs/PassMsg.h"
namespace msl
{

	class MSLWorldModel;
	class WhiteBoard
	{
	public:
		WhiteBoard(MSLWorldModel* wm);
		virtual ~WhiteBoard();
		void processPassMsg(msl_helper_msgs::PassMsgPtr msg);
		shared_ptr<msl_helper_msgs::PassMsg> getPassMsg(int index = 0);

	private:
		MSLWorldModel* wm;
		unsigned long maxInformationAge = 1000000000;
		shared_ptr<RingBuffer<InformationElement<msl_helper_msgs::PassMsg>>> passMsgs;
	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_WHITEBOARD_H_ */
