/*
 * $Id: RawOdometryHelper.h 1935 2007-03-19 19:50:12Z phbaer $
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
#ifndef RawOdometryHelper_H
#define RawOdometryHelper_H

#include <boost/thread/mutex.hpp>
#include "ros/ros.h"
#include <msl_actuator_msgs/RawOdometryInfo.h>
#include <string>

#include "../global/Types.h"

#define RAWODOBUFSIZE 100


class RawOdometryHelper{


	public:
		RawOdometryHelper();
		~RawOdometryHelper();

		void integrateData(Position pos, unsigned long long timestamp);
		Position getUpdateVectorAndReset();
		Position getUpdateVectorAndReset2();
		Position getPositionData();
		Position getPositionData(unsigned long long time);

		Position getVisionPos();
		Position getOdoPos();

		Position updatePositionWithOdoData(Position pos);

		ros::Subscriber sub;
		static RawOdometryHelper * getInstance();

		Point ego2AlloOnVision(Point p);
		Point allo2EgoOnVision(Point p);

		Point ego2AlloOnOdo(Point p);
		Point allo2EgoOnOdo(Point p);

		Point ego2Allo(Point p, Position pos);
		Point allo2Ego(Point p, Position pos);

		Velocity ego2Allo(Velocity vel, Position pos);
		Velocity allo2Ego(Velocity vel, Position pos);

		int getVisionIndex();
		int getOdoIndex();

		Position * getPositionBuffer();
		unsigned long long * getTimestampBuffer();

	protected:

		static RawOdometryHelper * instance_;
		
		void init();
		void cleanup();

		
		Position getPosDiffVector(Position posNew, Position posOld);
		Position updatePositionWithVector(Position pos, double deltaX, double deltaY, double deltaH, Position relPos);

		void handleRawOdometryInfo(const msl_actuator_msgs::RawOdometryInfo::ConstPtr& message);

		bool initialized;

		Position oldPosition;
		Position oldPosition2;
		Position newPosition;

		Position positionBuffer[RAWODOBUFSIZE];
		unsigned long long timestampBuffer[RAWODOBUFSIZE];
		//int visionIndex;
		int odoIndex;

		boost::mutex mutex;
};



#endif

