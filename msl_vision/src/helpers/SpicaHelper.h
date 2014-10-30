/*
 * $Id: Filter.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef SpicaHelper_h
#define SpicaHelper_h

#include "ros/ros.h"
#include <CNSensorMsgs/WorldModelData.h>
#include <CNSensorMsgs/VisionDebug.h>
#include <CNSensorMsgs/VisionControl.h>
#include <CNSensorMsgs/VisionImage.h>
#include <CNSensorMsgs/VisionGameState.h>
#include <CNActuatorMsgs/VisionRelocTrigger.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>



class SpicaHelper {
	public:
		static void initialize();
		static CNSensorMsgs::WorldModelData* wm;
		static CNSensorMsgs::VisionDebug* vdd;
		static CNSensorMsgs::VisionImage* vi;
		static bool reloc;

		static ros::Publisher womopub;
        static ros::Publisher statepub;
		static ros::Publisher LPpub;
		static ros::Publisher Imagepub;

		static ros::Subscriber VCsub;
		// subscriber for relocation button
		static ros::Subscriber RelocSub;
		static char key;
        static char haveBall;
        static char duel;


		static void handleVisionControl(const CNSensorMsgs::VisionControl::ConstPtr& msg);
		static void handleVisionRelocTrigger(const
				CNActuatorMsgs::VisionRelocTrigger::ConstPtr& msg);
		
		static ros::NodeHandle* visionNode;
		static void send();
		static void sendLinePoints();
        static void sendGameState();
		static void streamGreyMJPEG(unsigned char* img, int width, int height);


		static struct sockaddr_in si_other;
		static int s, slen;
		static unsigned char* buf;
};

#endif

