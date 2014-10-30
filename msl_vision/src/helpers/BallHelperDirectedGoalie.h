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
#ifndef BallHelperDirectedGoalie_H
#define BallHelperDirectedGoalie_H


#include "../filters/FilterExtractBlobsDirected.h"
#include "../global/Types.h"
#include "BallIntegrator.h"
#include "FootballField.h"
#include "ObjectTracker.h"
#include "TimeHelper.h"
#include "BallClusterHelp.h"
#include "RawOdometryHelper.h"
#include "PositionHelperDirected.h"
#include "ObjectContainer.h"

#define HEIGHT 480
#define WIDTH 640

class BallHelperDirectedGoalie{


	public:
		BallHelperDirectedGoalie();
		~BallHelperDirectedGoalie();
	
		Point getBallFromBlobs(ballCluster * cluster, int clusterCount, std::vector<ROIData>& roiData, std::vector<BlobBounds> & potBallBlobs);
		void visualizeBall(unsigned char * src, int width, Point ball, int radius);
		Point getBallPosition();

	protected:
		
		void init();
		void cleanup();

		ObservedPoint getBallFromBlobsAdaptiveROI(ballCluster * cluster, int clusterCount, std::vector<ROIData>& roiData, std::vector<BlobBounds> & potBallBlobs);

		double LocalizationSuccess;
		double * ballProbs;

		Point currBallPos;

		ObjectContainer ballBuf;

		MovingObject mv;

		SystemConfig* sc;

		ObservedPoint * observations;

		BlobBounds currBlob;
		BlobBounds trackBlob;
		BlobBounds velBlob;
		int velIterations;

		double blobVelX;
		double blobVelY;

		double trackBlobConfidence;
		int trackCounter;


};



#endif

