/*
 * $Id: Logger.cpp 2028 2007-04-11 19:28:28Z cn $
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
#include "Logger.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include <sys/time.h>
#include "SpicaHelper.h"
#include "TimeHelper.h"
#include "Environment.h"

Logger * Logger::instance_ = NULL;

Logger * Logger::getInstance(){

	if(instance_ == NULL)
		instance_ = new Logger();
	return instance_;


}


Logger::Logger() : mutex() {

	Environment * environment = Environment::getInstance();

	logfile = NULL;
	writeCounter = 1;

	//if(environment->getLoggingMode()){
	printf("Found logger file created\n");
	logfile = fopen(environment->getLogFileName().c_str(), "w");

	//}



	init();
}


Logger::~Logger(){

	if(logfile != NULL)
		fclose(logfile);

	cleanup();

}



void Logger::init(){


}


void Logger::cleanup(){


}


void Logger::logRawOdometry(Position pos, unsigned long long timestamp){

	boost::mutex::scoped_lock lock(this->mutex);

	if(!Environment::getInstance()->getLoggingMode())
		return;


	Position posTmp = pos;
	unsigned long long timeTmp = timestamp;
	unsigned char type = LOGTYPE_RAWODOMETRY;

	fwrite(&type, sizeof(unsigned char), 1, logfile);
	fwrite(&writeCounter, sizeof(int), 1, logfile);
	fwrite(&posTmp, sizeof(Position), 1, logfile);
	fwrite(&timeTmp, sizeof(unsigned long long), 1, logfile);


}

void Logger::logCompassValue(int value, unsigned long long timestamp){

	boost::mutex::scoped_lock lock(this->mutex);

	if(!Environment::getInstance()->getLoggingMode())
		return;


	int valueTmp = value;
	unsigned long long timeTmp = timestamp;
	unsigned char type = LOGTYPE_COMPASS;

	fwrite(&type, sizeof(unsigned char), 1, logfile);
	fwrite(&writeCounter, sizeof(int), 1, logfile);
	fwrite(&valueTmp, sizeof(int), 1, logfile);
	fwrite(&timeTmp, sizeof(unsigned long long), 1, logfile);


}

void Logger::logImageInfo(int imageNumber, unsigned long long timestamp){

	boost::mutex::scoped_lock lock(this->mutex);

	if(!Environment::getInstance()->getLoggingMode())
		return;


	int imageNumberTmp = imageNumber;
	unsigned long long timeTmp = timestamp;
	unsigned char type = LOGTYPE_IMAGE;

	fwrite(&type, sizeof(unsigned char), 1, logfile);
	fwrite(&writeCounter, sizeof(int), 1, logfile);
	fwrite(&imageNumberTmp, sizeof(int), 1, logfile);
	fwrite(&timeTmp, sizeof(unsigned long long), 1, logfile);


}


void Logger::setWriteCounter(int writeCounter_){

	boost::mutex::scoped_lock lock(this->mutex);
	writeCounter = writeCounter_;

}
