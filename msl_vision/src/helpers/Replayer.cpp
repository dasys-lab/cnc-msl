/*
 * $Id: Replayer.cpp 2028 2007-04-11 19:28:28Z cn $
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
#include "Replayer.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include <sys/time.h>
#include "SpicaHelper.h"
#include "TimeHelper.h"
#include "Environment.h"
#include "RawOdometryHelper.h"
#include "CompassValueHelper.h"

Replayer * Replayer::instance_ = NULL;

Replayer * Replayer::getInstance(){

	if(instance_ == NULL)
		instance_ = new Replayer();
	return instance_;


}


Replayer::Replayer() {

	Environment * environment = Environment::getInstance();

	logfile = NULL;
	lastImageTime = 0;

	//if(environment->getLoggingMode()){

	logfile = fopen(environment->getLogFileName().c_str(), "r");

	//}

	unsigned char type = 5;
	int writeCounter = 0;
	Position posTmp;
	unsigned long long time;

	int value = 0;
	int imageNumber = 0;


	while(!feof(logfile) && !ferror(logfile) && imageNumber != 1){	

		fread(&type, sizeof(unsigned char), 1, logfile);

		if(type == LOGTYPE_IMAGE){
			printf("Found Image LogEntry\n");

			fread(&writeCounter, sizeof(int), 1, logfile);
			fread(&imageNumber, sizeof(int), 1, logfile);
			fread(&time, sizeof(unsigned long long), 1, logfile);

			//imageNumber = *((int *) &posTmp);

			lastImageTime = time;

			printf("1234567891: Image %d %d %lld\n", writeCounter, imageNumber, time);

		} else 	if(type == LOGTYPE_RAWODOMETRY){
			printf("Found RawOdometry LogEntry\n");

			fread(&writeCounter, sizeof(int), 1, logfile);
			fread(&posTmp, sizeof(Position), 1, logfile);
			fread(&time, sizeof(unsigned long long), 1, logfile);

			printf("1234567891 %d %f %f %f %lld\n", writeCounter, posTmp.x, posTmp.y, posTmp.heading, time);

		} else 	if(type == LOGTYPE_COMPASS){


			fread(&writeCounter, sizeof(int), 1, logfile);
			fread(&value, sizeof(int), 1, logfile);
			fread(&time, sizeof(unsigned long long), 1, logfile);

			//value = *((int *) &posTmp);

			printf("Found Compass LogEntry\n");
			printf("1234567892 %d %d %lld\n", writeCounter, value, time);

		} else {
			printf("Unknown LogEntry\n");
		}


	}




	init();
}


Replayer::~Replayer(){

	if(logfile != NULL)
		fclose(logfile);

	cleanup();

}



void Replayer::init(){


}


void Replayer::cleanup(){


}


unsigned long long Replayer::replay(int currImage){

	unsigned char type = 5;
	int writeCounter = 0;
	Position posTmp;
	unsigned long long time;

	int value = 0;
	int imageNumber = 0;

	unsigned long long timeRet = 0;


	while(!feof(logfile) && !ferror(logfile) && imageNumber < currImage + 1){	

		fread(&type, sizeof(unsigned char), 1, logfile);

		if(type == LOGTYPE_IMAGE){
			printf("Found Image LogEntry\n");

			fread(&writeCounter, sizeof(int), 1, logfile);
			fread(&imageNumber, sizeof(int), 1, logfile);
			fread(&time, sizeof(unsigned long long), 1, logfile);

			//imageNumber = *((int *) &posTmp);

			timeRet = lastImageTime;
			lastImageTime = time;

			printf("1234567891: Image %d %d %lld\n", writeCounter, imageNumber, time);

		} else 	if(type == LOGTYPE_RAWODOMETRY){
			printf("Found RawOdometry LogEntry\n");

			fread(&writeCounter, sizeof(int), 1, logfile);
			fread(&posTmp, sizeof(Position), 1, logfile);
			fread(&time, sizeof(unsigned long long), 1, logfile);

			printf("1234567891 %d %f %f %f %lld\n", writeCounter, posTmp.x, posTmp.y, posTmp.heading, time);

			RawOdometryHelper::getInstance()->integrateData(posTmp, time);

		} else 	if(type == LOGTYPE_COMPASS){


			fread(&writeCounter, sizeof(int), 1, logfile);
			fread(&value, sizeof(int), 1, logfile);
			fread(&time, sizeof(unsigned long long), 1, logfile);

			//value = *((int *) &posTmp);

			printf("Found Compass LogEntry\n");
			printf("1234567892 %d %d %lld\n", writeCounter, value, time);
			CompassValueHelper::getInstance()->integrateData(value);

		} else {
			printf("Unknown LogEntry\n");
		}
	}


	return timeRet;

}

