/*
 * $Id: BallHelperDirected.h 1935 2007-03-19 19:50:12Z phbaer $
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
#ifndef BallHelper_H
#define BallHelper_H


#include "../filters/FilterExtractBlobs.h"
#include "../global/Types.h"
#include "ParticleFilter.h"
#include "SpicaHelper.h"
#include <msl_helper_msgs/PassMsg.h>
#include <msl_msgs/Point2dInfo.h>
#include "BallIntegrator.h"
#include "FootballField.h"
#include "ObjectTracker.h"
#include "TimeHelper.h"
#include "BallClusterHelp.h"
#include "RawOdometryHelper.h"
#include "PositionHelper.h"
#include "ObjectContainer.h"
//#include <DateTime.h>
#include "ros/ros.h"

using namespace msl_helper_msgs;

class BallHelper {

	public:
		BallHelper(int area);
		~BallHelper();
	
		void sendBallHypotesis(ballCluster * cluster, int clusterCount, std::vector<ROIData>& roiData);
		Point getBallFromBlobs(ballCluster * cluster, int clusterCount, std::vector<ROIData>& roiData, Particle * maxParticle);
		void visualizeBall(unsigned char * src, int width, Point ball, int radius);
		Point getBallPosition();
		void handlePassMessageInfo(const PassMsg::ConstPtr& message);

	protected:
			
		void init();
		void cleanup();

		ObservedPoint getBallFromBlobsAdaptiveROI(ballCluster * cluster, int clusterCount, std::vector<ROIData>& roiData, std::vector<BlobBounds> & potBallBlobs, Particle * maxParticle);


		ros::Subscriber sub;

		bool newOptics;
		double LocalizationSuccess;
		double * ballProbs;

		Point currBallPos;

		bool isGoalie;

		msl_msgs::Point2dInfo origin;
		msl_msgs::Point2dInfo destination;
		struct timeval passMsgTime;
		bool passMsgAvailable;

		ObjectContainer ballBuf;

		MovingObject mv;

		SystemConfig* sc;

		int MX;
		int MY;

		BlobBounds oldBlob;


};



#endif

