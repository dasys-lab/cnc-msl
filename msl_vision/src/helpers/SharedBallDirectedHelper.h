/*
 * $Id: SharedBallDirectedHelper.h 1935 2007-03-19 19:50:12Z phbaer $
 *
 *
 * Copyright 2009 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */
#ifndef SharedBallDirectedHelper_H
#define SharedBallDirectedHelper_H

#include <boost/thread/mutex.hpp>
#include <msl_sensor_msgs/SharedBallInfo.h>
#include <string>
#include "ros/ros.h"

#include "../global/Types.h"

using namespace msl_sensor_msgs;


class SharedBallDirectedHelper{

	public:
		SharedBallDirectedHelper();
		~SharedBallDirectedHelper();

		static SharedBallDirectedHelper *getInstance();
		SharedBall getBall();

	protected:

		void init();
		void cleanup();

		boost::mutex mutex;
		SharedBall ball;
		ros::Subscriber sub;

		void handleSharedBallInfo(const SharedBallInfo::ConstPtr& message);

		bool initialized;

		static SharedBallDirectedHelper *instance;
};



#endif

