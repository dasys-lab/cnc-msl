/*
 * $Id: Replayer.h 1935 2007-03-19 19:50:12Z phbaer $
 *
 *
 * Copyright 2005,2006 Carpe Noctem, Distributed Systems Group,
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
#ifndef Replayer_H
#define Replayer_H

#include <boost/thread/mutex.hpp>
#include <msl_actuator_msgs/RawOdometryInfo.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>

#include "../global/Types.h"
#include "Logger.h"


class Replayer{


	public:

		~Replayer();

		unsigned long long replay(int currImage);

		static Replayer * getInstance();


	protected:

		Replayer();

		static Replayer * instance_;

		void init();
		void cleanup();

		unsigned long long lastImageTime;

		FILE * logfile;

		//boost::mutex mutex;
};



#endif

