/*
 * $Id: BallHelperKinect.h 1935 2007-03-19 19:50:12Z phbaer $
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
#ifndef BallHelperKinect_H
#define BallHelperKinect_H

#include <vector>
#include <queue>
#include <stdint.h>

#include "../global/Types.h"
#include "ObjectContainer.h"
#include "SystemConfig.h"

// Resolution that is used here. Kinect resolution is 640x480 (doubled)
#define FRAME_W 320
#define FRAME_H 240
#define FRAME_PIX (FRAME_H*FRAME_W)

// Cluster IDs for nice, colorful display of clusters
#define BALL_CLUSTER 1337
#define BALL_CENTER 1336
#define BALL_NSWE 1335

class BallHelperKinect {
	public:
		BallHelperKinect();
		~BallHelperKinect();

		void assignCluster(uint16_t*);
		Point getBallCluster();
		Point getSimpleBallCluster();

		void doubleRes();

		void visualizeBall(unsigned char * src, int width, Point ball,
				int radius);

		inline std::vector<int>* getMatrix() {
			return &m_cluster;
		}

		inline std::vector<KinectCluster*>* getClustering() {
			return &clustering;
		}

	protected:
		double LocalizationSuccess;

		ObjectContainer ballBuf;
		MovingObject mv;
		supplementary::SystemConfig* sc;
		ObservedPoint * observations;

	private:
		int centerX;
		int centerY;
		int camX;
		int camY;
		int camZ;
		int threshold;
		unsigned minClusterSize;
		double widthHeightRatio;
		double invertedWidthHeightRatio;
		int ballSize;
		int ballRadius;
		int maxBallSizeDiff;
		int maxEdgeDistDiff;
		int lowerBound;
		int upperBound;
		int virtualScreenDistance;
		double distPerPixel;

		std::vector<KinectCluster*> clustering;
		std::vector<int> m_cluster;
		uint16_t* depth;
		std::queue<int> queue;

		double depth2Distance(int);
		int distance2Depth(double);
		int calcThreshold(int);

		int col(int);
		int row(int);

		void calcSize(KinectCluster*);
		double calcRatio(double, double);
		void calculateRelations(KinectCluster*, int, int, int, int);
		void setCamToPoint3D(KinectCluster*);
		Point point3DToCam(int, int, int);
		Point integrate(bool, ObservedPoint*, int);

		void paint(int, int);
};

#endif

