/*
 * $Id: Packets.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef Packets_H
#define Packets_H

#define EPOCH_ADJUST ((unsigned long long)62135596800LL)

#define MAX_SCANLENGTH 180

#define BALLPACKET_TYPE 1
#define DISTANCESCAN_TYPE 6

struct PacketHeader
{

    char endian;
    char version;
    char origin;
    char type;
    unsigned long long timestamp;
};

struct BallPacket
{

    PacketHeader header;
    double x;
    double y;
};

struct DistanceScanPacket
{

    PacketHeader header;
    unsigned int length;
    double values[MAX_SCANLENGTH];
};

struct OdometryPacket
{

    PacketHeader header;
    double x;
    double y;
    double heading;
    double angle;
    double translation;
    double rotation;
};

#endif
