/*
 * $Id: Logger.h 1935 2007-03-19 19:50:12Z phbaer $
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
#ifndef Logger_H
#define Logger_H

#include <boost/thread/mutex.hpp>
#include <CNActuatorMsgs/RawOdometryInfo.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>

#include "../global/Types.h"


#define LOGTYPE_IMAGE 0
#define LOGTYPE_RAWODOMETRY 1
#define LOGTYPE_COMPASS 2


class Logger{


	public:

		~Logger();

		void logRawOdometry(Position pos, unsigned long long timestamp);
		void logCompassValue(int value, unsigned long long timestamp);
		void logImageInfo(int imageNumber, unsigned long long timestamp);
		void setWriteCounter(int writeCounter_);

		static Logger * getInstance();


	protected:

		Logger();

		static Logger * instance_;
		
		void init();
		void cleanup();

		int writeCounter;

		FILE * logfile;

		boost::mutex mutex;
};



#endif

