/*
 * WhiteBoard.cpp
 *
 *  Created on: Aug 28, 2015
 *      Author: Stefan Jakob
 */

#include <WhiteBoard.h>
#include "MSLWorldModel.h"

namespace msl
{

	WhiteBoard::WhiteBoard(MSLWorldModel* wm)
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
		shared_ptr<InformationElement<msl_helper_msgs::PassMsg>> passMsg = make_shared<
				InformationElement<msl_helper_msgs::PassMsg>>(p, time);
		passMsgs->add(passMsg);
	}

	shared_ptr<msl_helper_msgs::PassMsg> WhiteBoard::getPassMsg(int index)
	{
		auto x = passMsgs->getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

} /* namespace msl */

