/*
 * $Id: PacketHelper.cpp 1935 2007-03-19 19:50:12Z phbaer $
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
#include "PacketHelper.h"
/*
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

void PacketHelper::initHeader(PacketHeader & header, int type){

	header.endian = ENDIANESS;
	header.version = VERSION;
	header.origin = ORIGIN;
	header.type = type;

	struct timeval tv;
	gettimeofday(&tv, NULL);
	header.timestamp =  (((unsigned long long)tv.tv_sec + EPOCH_ADJUST)* 1000000 + tv.tv_usec)*10;

}


void PacketHelper::initHeaderMDT(CarpeNoctem::Header * header, CarpeNoctem::EventHeader * manHeader, unsigned short msgid){

	manHeader->setOrigin(ORIGIN);

	struct timeval tv;
	gettimeofday(&tv, NULL);
	manHeader->setTimestamp((((unsigned long long)tv.tv_sec + EPOCH_ADJUST)* 1000000 + tv.tv_usec)*10);

	header->setMsgid(msgid);

}
*/

