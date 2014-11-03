/*
 * $Id: BallHelperKinect.cpp 1935 2007-03-19 19:50:12Z phbaer $
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
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>

#include <libfreenect_sync.h>

#include "BallHelperKinect.h"
#include "SharedMemoryHelper.h"
#include "RawOdometryHelper.h"
#include "BallIntegrator.h"
#include "FootballField.h"
#include "ObjectTracker.h"
#include "TimeHelper.h"

// Maximum value that can appear in a depth image (11-bit values)
// Pixels with this value represent invalid data (due to shadows etc.)
#define MAX_DEPTH 2047

using namespace std;

BallHelperKinect::BallHelperKinect() : ballBuf(30),
		m_cluster(4*FRAME_PIX) {
	this->sc = SystemConfig::getInstance();
	FootballField::getInstance();

	LocalizationSuccess = (*this->sc)["Localization"]->get<double>(
			"Localization", "LocalizationSuccess", NULL);

	const char* vision = "Vision";
	const char* kinect = "KinectSettings";
	centerX = (*this->sc)[vision]->get<int>(vision, kinect, "CenterX",
			NULL);
	centerY = (*this->sc)[vision]->get<int>(vision, kinect, "CenterY",
			NULL);
	camX = (*this->sc)[vision]->get<int>(vision, kinect, "CamX", NULL);
	camY = (*this->sc)[vision]->get<int>(vision, kinect, "CamY", NULL);
	camZ = (*this->sc)[vision]->get<int>(vision, kinect, "CamZ", NULL);
	threshold = (*this->sc)[vision]->get<int>(vision, kinect,
			"Threshold", NULL);
	minClusterSize = (*this->sc)[vision]->get<unsigned>(vision, kinect,
			"MinClusterSize", NULL);
	widthHeightRatio = (*this->sc)[vision]->get<double>(vision, kinect,
			"WidthHeightRatio", NULL);
	invertedWidthHeightRatio = (*this->sc)[vision]->get<double>(vision,
			kinect, "InvertedWidthHeightRatio", NULL);
	ballSize = (*this->sc)[vision]->get<int>(vision, kinect, "BallSize",
			NULL);
	ballRadius = (*this->sc)[vision]->get<int>(vision, kinect,
			"BallRadius", NULL);
	maxBallSizeDiff = (*this->sc)[vision]->get<int>(vision, kinect,
			"MaxBallSizeDiff", NULL);
	maxEdgeDistDiff = (*this->sc)[vision]->get<int>(vision, kinect,
			"MaxEdgeDistDiff", NULL);
	lowerBound = (*this->sc)[vision]->get<int>(vision, kinect,
			"LowerBound", NULL);
	upperBound = (*this->sc)[vision]->get<int>(vision, kinect,
			"UpperBound", NULL);
	virtualScreenDistance = (*this->sc)[vision]->get<int>(vision,
			kinect, "VirtualScreenDistance", NULL);
	distPerPixel = (*this->sc)[vision]->get<double>(vision, kinect,
			"DistPerPixel", NULL);

	observations = (ObservedPoint *) malloc(10*sizeof(ObservedPoint));

	mv.point.x = 0.0;
	mv.point.y = 0.0;
	mv.velocity.vx = 0.0;
	mv.velocity.vy = 0.0;

	depth = new unsigned short[FRAME_PIX];
}

BallHelperKinect::~BallHelperKinect(){
	delete depth;
}

void BallHelperKinect::visualizeBall(unsigned char * src, int width,
		Point ball, int radius) {
	//cout << "Endy Endy2CamPos x:" << ball.x << " y:" << ball.y << endl;
	int index = ((int)ball.x) + ((int)ball.y)*width;
	src[index] = 255;

	src[index+1] = 150;
	src[index-1] = 150;
	src[index+2] = 150;
	src[index-2] = 150;
	src[index+3] = 150;
	src[index-3] = 150;

	src[index+1*width] = 150;
	src[index-1*width] = 150;

	src[index+2*width] = 150;
	src[index-2*width] = 150;

	src[index+3*width] = 150;
	src[index-3*width] = 150;
}

///////////////////////////////////////////////////////////////////////////

// Calculates the distance in mm via depth value
inline double BallHelperKinect::depth2Distance(int depth) {
	if (depth > 1091)
		return 0.0;
	return 348000.0 / (1091.5 - depth);
}

// Calculates the depth value via the distance in mm
inline int BallHelperKinect::distance2Depth(double dist) {
	if (dist <= 0)
		return MAX_DEPTH;
	return (int) (1091.5 - 348000.0 / dist);
}

// Calculates the threshold for noise suppression
inline int BallHelperKinect::calcThreshold(int depth) {
	if (depth < 1030)
		return distance2Depth(depth2Distance(depth) + threshold) - depth;
	return distance2Depth(depth2Distance(depth) + 16 * depth - 16280) - depth;
}

inline int BallHelperKinect::col(int index) {
	return index % FRAME_W;
}

inline int BallHelperKinect::row(int index) {
	return index / FRAME_W;
}

// assigns every point in matrix a clusterid
void BallHelperKinect::assignCluster(uint16_t* depth_big) {
	// Halve resolution
	int row = -1;
	int index_b;
	for (int index_s = 0; index_s < FRAME_PIX; ++index_s) {
		if (col(index_s) == 0)
			++row;
		index_b = 2 * index_s + row * 2*FRAME_W;

		depth[index_s] = (depth_big[index_b] + depth_big[index_b + 1]
				+ depth_big[index_b + 2*FRAME_W]
				+ depth_big[index_b + 2*FRAME_W + 1]) / 4.0 + 0.5;
	}

	// Delete clustering from last frame
	for (unsigned i = 0; i < clustering.size(); ++i)
		delete clustering[i];
	clustering.clear();

	// Init new clustering and cluster matrix
	KinectCluster* c = new KinectCluster();
	c->clusterID = 0;
	clustering.push_back(c);
	int clusterCounter = 0;
	for (int i = 0; i < FRAME_PIX; ++i)
		m_cluster[i] = -1;

	for (int i = 0; i < FRAME_PIX; ++i) {
		// If not visited
		if (m_cluster[i] == -1) {
			// Check if data is available
			if (depth[i] != MAX_DEPTH) {
				queue.push(i);
				KinectCluster* c = new KinectCluster();
				c->clusterID = ++clusterCounter;
				clustering.push_back(c);

				while (!queue.empty()) {
					int current = queue.front();
					queue.pop();

					if (m_cluster[current] != -1)
						continue;

					int t = calcThreshold(depth[current]);

					int west = current - 1;
					while (col(west + 1) != 0 && m_cluster[west] == -1
							&& depth[west] != MAX_DEPTH
							&& abs(depth[west] - depth[west + 1]) < t)
						west--;

					int east = current + 1;
					while (col(east) != 0 && m_cluster[east] == -1
							&& depth[east] != MAX_DEPTH
							&& abs(depth[east] - depth[east - 1]) < t)
						east++;

					for (int j = west + 1; j < east; ++j) {
						m_cluster[j] = clusterCounter;
						clustering[clusterCounter]->addPixel(j);

						int north = j - FRAME_W;
						int south = j + FRAME_W;

						if (north >= 0 && m_cluster[north] == -1
								&& depth[north] != MAX_DEPTH
								&& abs(depth[north] - depth[j]) < t)
							queue.push(north);
						if (south < FRAME_PIX && m_cluster[south] == -1
								&& depth[south] != MAX_DEPTH
								&& abs(depth[south] - depth[j]) < t)
							queue.push(south);
					}
				}
			} else {
				// No data available -> assign cluster 0
				m_cluster[i] = 0;
			}
		}
	}
}

/*
 * Examines the clusters found by the assignCluster method. If they are
 * likely to be a ball they are handed over to the BallIntegrator and written
 * to shared memory
 */
Point BallHelperKinect::getBallCluster() {
	ObservedPoint ballPos;
	bool validBalls = false;
	int sharedMemoryIndex = 0;

	ballPos.x = 0.0;
	ballPos.y = 0.0;
	ballPos.z = 0.0;
	ballPos.confidence = 0.0;
	ballPos.valid = false;

	int condCounter[12] = { 0 };
	string condNames[12];
	condNames[0] = "too less px      ";
	condNames[1] = "is hole          ";
	condNames[2] = "width/height     ";
	condNames[3] = "invW / invH      ";
	condNames[4] = "too small        ";
	condNames[5] = "too big          ";
	condNames[6] = "small radius     ";
	condNames[7] = "big radius       ";
	condNames[8] = "radius avg       ";
	condNames[9] = "off field (ego)  ";
	condNames[10] = "off field (allo) ";
	condNames[11] = "found ball       ";

	for (unsigned i = 1; i < clustering.size(); ++i) {
		KinectCluster* current = clustering[i];

		// Ignore small clusters
		if (current->pixels.size() < minClusterSize) {
			condCounter[0]++;
			continue;
		}

		calcSize(current);
		double w = current->width;
		double h = current->height;
		double invW = current->invWidth;
		double invH = current->invHeight;

		// Ignore clusters whose surrounding is closer than the cluster
		bool north = current->north >= FRAME_W
				&& depth[current->north] < depth[current->north - FRAME_W];
		// Due to the construction of the Kinect there is always a few pixels
		// with undefined values on the left of objects
		bool west = col(current->west) > 0
				&& (depth[current->west] < depth[current->west - 1]
				|| depth[current->west - 1] == MAX_DEPTH);
		bool east = col(current->east+1) > 0
			&& depth[current->east] < depth[current->east + 1];
		// Does not check the bottom border, to be able to recognize balls
		// lying on the ground or similar
		if(north || west || east ) {
			//condCounter[1]++;
			//continue;
		}
		
		// Ignore clusters that are not approximately as high as they are wide
		if (calcRatio(h, w) < widthHeightRatio) {
			//condCounter[2]++;
			//continue;
		}

		// Ignore clusters that are leaning over somehow
		if (calcRatio(invW, w) > invertedWidthHeightRatio
				|| calcRatio(invH, h) > invertedWidthHeightRatio) {
			//condCounter[3]++;
			//continue;
		}

		// Calculate coordinates of and distance to the center
		int x = (col(current->east) + col(current->west)) / 2.0 + 0.5;
		int y = (row(current->north) + row(current->south)) / 2.0 + 0.5;
		int center = y * FRAME_W + x;
		int dist = depth2Distance(depth[center]);

		// Width and height on the virtual screen
		int virtualWidth = w * dist / virtualScreenDistance;
		int virtualHeight = h * dist / virtualScreenDistance;

		// Ignore clusters that are too small
		if (virtualWidth < ballSize - maxBallSizeDiff
				|| virtualHeight < ballSize - maxBallSizeDiff) {
			condCounter[4]++;
			continue;
		}

		// Ignore clusters that do not have at least on dimension within the
		// threshold
		if (virtualWidth > ballSize + maxBallSizeDiff
				&& virtualHeight > ballSize + maxBallSizeDiff) {
			condCounter[5]++;
			continue;
		}

		// Calc edge distances
		double distanceSum = 0;
		int count = 0;
		bool skip = false;
		bool skip2 = false;
		for (unsigned j = 0; j < current->pixels.size(); ++j) {
			int pixel = current->pixels[j];

			// Skip if all surrounding pixels belong to the same cluster
			bool left = col(pixel) == 0
				|| m_cluster[pixel - 1] == m_cluster[pixel];
			bool right = col(pixel + 1) == 0
				|| m_cluster[pixel + 1] == m_cluster[pixel];
			bool top = pixel < FRAME_W
				|| m_cluster[pixel - FRAME_W] == m_cluster[pixel];
			bool bottom = pixel + FRAME_W >= FRAME_PIX
				|| m_cluster[pixel + FRAME_W] == m_cluster[pixel];

			int cond = left + right + top + bottom;
			if (cond > 3)
				continue;

			double x = col(pixel) - col(center);
			double y = row(pixel) - row(center);
			double distToCenter = sqrt(x*x + y*y) * distPerPixel * dist
					/ virtualScreenDistance;

			if (distToCenter < ballRadius - maxEdgeDistDiff) {
				skip = true;
				break;
			}

			if (distToCenter > 2 * ballRadius + maxEdgeDistDiff) {
				skip2 = true;
				break;
			}

			distanceSum += distToCenter;
			++count;
		}
		if (skip) {
			condCounter[6]++;
			continue;
		}
		if (skip2) {
			condCounter[7]++;
			continue;
		}

		distanceSum = distanceSum / count;
		// TODO *2 ?
		if (distanceSum < ballRadius - maxEdgeDistDiff / 2
				|| distanceSum > 2 * ballRadius + maxEdgeDistDiff / 2) {
			//condCounter[8]++;
			//continue;
		}

		current->center = center;
		setCamToPoint3D(current);

		// Ignore clusters outside of the field (own half only, ego coordinates)
		double relFactor = 2000;
		if(current->position.x < -(FootballField::FieldLength/2 + relFactor)
				|| abs(current->position.y) > FootballField::FieldWidth / 2
				+ relFactor || current->position.z < -200) {
			condCounter[9]++;
			continue;
		}

		// Ignore clusters outside of the field (own half only, allo coords)
		CorrectedOdometry* alloPos = SharedMemoryHelper::getInstance()->
				readCorrectedOdometry();
		if(alloPos != NULL && alloPos->posCertainty >= LocalizationSuccess){
			double alloX = alloPos->posX; //maxParticle->posx;
			double alloY = alloPos->posY; //maxParticle->posy;

			alloX += cos(alloPos->posAngle)*current->position.x
					- sin(alloPos->posAngle)*current->position.y;
			alloY += sin(alloPos->posAngle)*current->position.x
					+ cos(alloPos->posAngle)*current->position.y;

			cout << "Allo Pos: " << alloPos->posX << ", " << alloPos->posY
					<< ", " << alloPos->posAngle << endl;
			cout << "Ego Ball: " << current->position.x << ", "
					<< current->position.y << endl;
			cout << "Allo Ball: " << alloX << ", " << alloY << endl;

			relFactor = 200;
			if(abs(alloX) > FootballField::FieldLength / 2
					|| abs(alloPos->posX - alloX)
						> FootballField::FieldLength / 2 + relFactor
					|| abs(alloY) > FootballField::FieldWidth/2 + relFactor) {
				condCounter[10]++;
				continue;
			}
		}

		//
		// Ball found
		//
		condCounter[11]++;
		current->confidence = 0.9 * calcRatio(h, w) * (1 - 	pow(calcRatio(
				invW, w), 2)) * (1 - pow(calcRatio(invH, h), 2));
		
		for (unsigned j = 0; j < current->pixels.size(); ++j)
			m_cluster[current->pixels[j]] = BALL_CLUSTER;

		// paint the center of the ball
		paint(center, BALL_CENTER);

		// paint n,s,e,w
		paint(current->north, BALL_NSWE);
		paint(current->south, BALL_NSWE);
		paint(current->east, BALL_NSWE);
		paint(current->west, BALL_NSWE);


		Point3D* pos = &current->position;
		cout << endl << "**********************************************" <<endl;
		cout << "Width:\t\t\t" << w << " px -> " << virtualWidth << " px"<<endl;
		cout << "Height:\t\t\t"<< h << " px -> " << virtualHeight <<" px"<<endl;
		cout << "Pixelcount:\t\t" << current->pixels.size() << endl;
		cout << "Depth:\t\t\t" << depth[center] << " -> " << dist <<" mm"<<endl;
		cout << "Width Height Ratio:\t" << calcRatio(h, w) << endl;
		cout << "InvWidth Ratio:\t\t" << calcRatio(invW, w) << endl;
		cout << "InvHeight Ratio:\t\t" << calcRatio(invH, h) << endl;
		cout << "Position:\tx:\t" << pos->x << " mm" << endl << "\t\ty:\t"
				<< pos->y << " mm" << endl << "\t\tz:\t"<< pos->z <<" mm"<<endl;
		cout << "Avg Distance:\t\t" << distanceSum << " mm" << endl;
		cout << "Confidence:\t\t" << current->confidence << endl;
		cout << "**********************************************" << endl <<endl;

		ballPos.x = pos->x;
		ballPos.y = pos->y;
		ballPos.z = pos->z;
		ballPos.valid = true;
		ballPos.confidence = current->confidence;
		// TODO omniCam?
		ballPos.timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();

		BallIntegrator::getInstance()->integratePoint(ballPos, 1000.0);
		validBalls = true;

		if(sharedMemoryIndex < 10) {
			observations[sharedMemoryIndex] = ballPos;
			sharedMemoryIndex++;
		}
	}

#ifdef VERBOSE
	for (int j = 0; j < 12; ++j)
		cout << "Filter: " << condNames[j] << ": " << condCounter[j] << endl;
#endif

	return integrate(validBalls, observations, sharedMemoryIndex);
}

/*
 * Does basically the same as the getBallCluster method, but uses a lot
 * less filters.
 * The only filters applied here are if the cluster is within the field
 * boundaries, up in the air and not to big to be a ball.
 */
Point BallHelperKinect::getSimpleBallCluster() {
	ObservedPoint ballPos;
	bool validBalls = false;
	int sharedMemoryIndex = 0;

	ballPos.x = 0.0;
	ballPos.y = 0.0;
	ballPos.z = 0.0;
	ballPos.confidence = 0.0;
	ballPos.valid = false;

	int condCounter[7] = { 0 };
	string condNames[7];
	condNames[0] = "too less px     ";
	condNames[1] = "far too big     ";
	condNames[2] = "too big         ";
	condNames[3] = "not field (ego) ";
	condNames[4] = "not in air      ";
	condNames[5] = "not field (allo)";
	condNames[6] = "found ball      ";

	for (unsigned i = 1; i < clustering.size(); ++i) {
		KinectCluster* current = clustering[i];

		// Ignore small clusters
		if (current->pixels.size() < minClusterSize) {
			condCounter[0]++;
			continue;
		}

		calcSize(current);
		//printf("current north %i - south %i - west %i - east %i\n", current->north, current->south, current->west, current->east);
		//printf("current height %i - width %i\n", current->height, current->width);
		//printf("current invHeight %i - invWidth %i - center %i\n", current->invHeight, current->invWidth, current->center);
		//printf("current x %i - y %i - z %i\n", (int)current->position.x, (int)current->position.y, (int)current->position.z);
		double w = current->width;
		double h = current->height;
		double invW = current->invWidth;
		double invH = current->invHeight;

		// Calculate coordinates of and distance to the center
		int x = (col(current->east) + col(current->west)) / 2.0 + 0.5;
		int y = (row(current->north) + row(current->south)) / 2.0 + 0.5;
		int center = y * FRAME_W + x;
		int dist = depth2Distance(depth[center]);

		// Width and height on the virtual screen
		int virtualWidth = w * dist / virtualScreenDistance;
		int virtualHeight = h * dist / virtualScreenDistance;

		// Ignore clusters that are a far too big
		if (virtualWidth > 2 * ballSize + maxBallSizeDiff
				|| virtualHeight > 2 * ballSize + maxBallSizeDiff) {
			condCounter[1]++;
			continue;
		}

		// Ignore clusters that do not have at least on dimension within the
		// size limits
		if (virtualWidth > ballSize + maxBallSizeDiff
				&& virtualHeight > ballSize + maxBallSizeDiff) {
			condCounter[2]++;
			continue;
		}

		current->center = center;
		setCamToPoint3D(current);
		/*current->position.x*=10;		
		current->position.y*=10;
		current->position.z*=10;*/

		// Ignore clusters outside of the field (own half only, ego coordinates)
		double relFactor = 2000;
		if(current->position.x < -(FootballField::FieldLength/2 + relFactor)
				|| abs(current->position.y) > FootballField::FieldWidth / 2
				+ relFactor) {
			condCounter[3]++;
			continue;
		}

		cout << "ValidBalls Pos Z: " << current->position.z << " Bound: " << lowerBound << " : " << upperBound << endl; 
		// Ignore clusters that are not in the air or too high
		printf("current x %d - y %d - z %d\n", current->position.x, current->position.y, current->position.z);
		if(current->position.z < lowerBound
				|| current->position.z > upperBound ) {
			condCounter[4]++;
			continue;
		}

		// Ignore clusters outside of the field (own half only, allo coords)
		CorrectedOdometry* alloPos = SharedMemoryHelper::getInstance()->
				readCorrectedOdometry();
		if(alloPos != NULL && alloPos->posCertainty >= LocalizationSuccess){
			double alloX = alloPos->posX; //maxParticle->posx;
			double alloY = alloPos->posY; //maxParticle->posy;

			alloX += cos(alloPos->posAngle)*current->position.x
					- sin(alloPos->posAngle)*current->position.y;
			alloY += sin(alloPos->posAngle)*current->position.x
					+ cos(alloPos->posAngle)*current->position.y;

			cout << "Allo Pos: " << alloPos->posX << ", " << alloPos->posY
					<< ", " << alloPos->posAngle << endl;
			cout << "Ego Ball: " << current->position.x << ", "
					<< current->position.y << endl;
			cout << "Allo Ball: " << alloX << ", " << alloY << endl;

			relFactor = 200;
			if(abs(alloX) > FootballField::FieldLength / 2
					|| abs(alloPos->posX - alloX)
						> FootballField::FieldLength / 2 + relFactor
					|| abs(alloY) > FootballField::FieldWidth/2 + relFactor) {
				condCounter[5]++;
				continue;
			}
		}

		//
		// Ball found
		//
		condCounter[6]++;
		current->confidence = 0.9 * calcRatio(h, w);
		
		for (unsigned j = 0; j < current->pixels.size(); ++j)
			m_cluster[current->pixels[j]] = BALL_CLUSTER;

		// paint the center of the ball
		paint(center, BALL_CENTER);

		// paint n,s,e,w
		paint(current->north, BALL_NSWE);
		paint(current->south, BALL_NSWE);
		paint(current->east, BALL_NSWE);
		paint(current->west, BALL_NSWE);


		Point3D* pos = &current->position;
		cout << endl << "**********************************************" <<endl;
		cout << "Width:\t\t\t" << w << " px -> " << virtualWidth << " px"<<endl;
		cout << "Height:\t\t\t"<< h << " px -> " << virtualHeight <<" px"<<endl;
		cout << "Pixelcount:\t\t" << current->pixels.size() << endl;
		cout << "Depth:\t\t\t" << depth[center] << " -> " << dist <<" mm"<<endl;
		cout << "Width Height Ratio:\t" << calcRatio(h, w) << endl;
		cout << "InvWidth Ratio:\t\t" << calcRatio(invW, w) << endl;
		cout << "InvHeight Ratio:\t\t" << calcRatio(invH, h) << endl;
		cout << "Position:\tx:\t" << pos->x << " mm" << endl << "\t\ty:\t"
				<< pos->y << " mm" << endl << "\t\tz:\t"<< pos->z <<" mm"<<endl;
		cout << "Confidence:\t\t" << current->confidence << endl;
		cout << "**********************************************" << endl <<endl;

		ballPos.x = pos->x;
		ballPos.y = pos->y;
		ballPos.z = pos->z;
		ballPos.valid = true;
		ballPos.confidence = current->confidence;
		// TODO omniCam?
		ballPos.timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();

		BallIntegrator::getInstance()->integratePoint(ballPos, 1000.0);
		validBalls = true;

		if(sharedMemoryIndex < 10) {
			observations[sharedMemoryIndex] = ballPos;
			sharedMemoryIndex++;
		}
	}

	cout << "ValidBalls: "  << validBalls << endl;
#ifdef VERBOSE
	for (int j = 0; j < 7; ++j)
		cout << "Filter: " << condNames[j] << ": " << condCounter[j] << endl;
#endif

	return integrate(validBalls, observations, sharedMemoryIndex);
}

Point BallHelperKinect::integrate(bool validBalls, ObservedPoint* observations,
		int sharedMemoryIndex) {
	// No valid balls found - integrate dummy
	if(!validBalls) {
		freenect_sync_set_led(LED_RED, 0);
		ObservedPoint ballPos;
		ballPos.x = 0.0;
		ballPos.y = 0.0;
		ballPos.z = 0.0;
		ballPos.confidence = 0.0;
		ballPos.valid = false;

		BallIntegrator::getInstance()->integratePoint(ballPos, 1000.0);
	} else
		freenect_sync_set_led(LED_GREEN, 0);

	BallIntegrator::getInstance()->decreaseDirtyPointCertainty();
	// Invalidate remaining indices
	for(int i = sharedMemoryIndex; i < 10; i++)
		observations[i].valid = false;

	cout << "SharedMemoryHelper: wrote " <<sharedMemoryIndex<< " balls" <<endl;
	SharedMemoryHelper::getInstance()->writeKinectBallPosition(observations);
	ObservedPoint op = BallIntegrator::getInstance()->getPoint();
	// TODO omniCam?
	op.timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();

	double dist = sqrt( pow(op.x-mv.point.x,2) + pow(op.y-mv.point.y,2) );
	if(dist > 750.0)
		ballBuf.reset();

	ballBuf.integratePoint(op);
	ballBuf.invalidate(400);

	mv = ObjectTracker::getInstance()->trackObject(ballBuf.getPoints(),
			ballBuf.getSize(), ballBuf.getStartIndex(), ballBuf.getLastIndex(),
			0.3E07);
	RawOdometryHelper * rawHelper = RawOdometryHelper::getInstance();

	MovingObject mv2 = mv;
	mv2.point = rawHelper->allo2EgoOnVision(mv.point);
	mv2.velocity = rawHelper->allo2Ego(mv.velocity, rawHelper->getVisionPos());

	Point ballImPoint = point3DToCam(mv2.point.x, mv2.point.y, op.z);

	if(ballImPoint.x < 17) ballImPoint.x = 17;
	if(ballImPoint.x > FRAME_W - 17) ballImPoint.x = FRAME_W - 17;
	if(ballImPoint.y < 17) ballImPoint.y = 17;
	if(ballImPoint.y > FRAME_H - 17) ballImPoint.y = FRAME_H - 17;

	cout << "Endy Elm2CamPos x:" << ballImPoint.x << " y:"<<ballImPoint.y<<endl;
	cout << "BallPos x:" << mv.point.x << " y:" << mv.point.y << " X-Vel: "
		<< mv.velocity.vx << " Y-Vel: " << mv.velocity.vy << endl;
	cout << "BallPos X-Vel: " << mv2.velocity.vx << "\tY-Vel: "
		<< mv2.velocity.vy << endl;
	cout << "BallTracker - BallContainer - startIndex = "
		<< ballBuf.getStartIndex() << " lastIndex = " << ballBuf.getLastIndex()
		<< endl;
	cout << "BallTracker - MovingObject - " << mv.point.x << " " << mv.point.y
		<< " " << mv.velocity.vx << " " << mv.velocity.vy << endl;

	// Paint center of camera view (origin of coordinates)
	paint(centerY * FRAME_W + centerX, BALL_NSWE);

	return ballImPoint;
}

inline void BallHelperKinect::paint(int index, int color) {
	m_cluster[index] = color;
	m_cluster[index - 1] = color;
	m_cluster[index + 1] = color;
	m_cluster[index - FRAME_W] = color;
	//m_cluster[index - FRAME_W - 1] = color;
	//m_cluster[index - FRAME_W + 1] = color;
	m_cluster[index + FRAME_W] = color;
	//m_cluster[index + FRAME_W - 1] = color;
	//m_cluster[index + FRAME_W + 1] = color;
}

inline double BallHelperKinect::calcRatio(double height, double width) {
	return min(height, width) / max(height, width);
}

// Calculates the max width and height of a cluster
// Assumes that there is at least one pixel in the cluster
void BallHelperKinect::calcSize(KinectCluster* cluster) {
	int north = cluster->pixels[0];
	int west = cluster->pixels[0];
	int south = cluster->pixels[0];
	int east = cluster->pixels[0];

	for (unsigned i = 1; i < cluster->pixels.size(); ++i) {
		if (cluster->pixels[i] < north)
			north = cluster->pixels[i];
		else if (cluster->pixels[i] > south)
			south = cluster->pixels[i];
		if (col(cluster->pixels[i]) < col(west))
			west = cluster->pixels[i];
		else if (col(cluster->pixels[i]) > col(east))
			east = cluster->pixels[i];
	}

	calculateRelations(cluster, north, south, west, east);
}

void BallHelperKinect::setCamToPoint3D(KinectCluster* c) {
	int center = c->center;
	double dist = depth2Distance(depth[center]) + ballRadius;
	double wDiff = col(center) - centerX;
	double hDiff = centerY - row(center);

	c->position.y = wDiff * distPerPixel * dist / virtualScreenDistance;
	c->position.z = hDiff * distPerPixel * dist / virtualScreenDistance;
	c->position.x = -sqrt( dist*dist - c->position.y*c->position.y
			- c->position.z*c->position.z );

	c->position.x += camX;
	c->position.y += camY;
	c->position.z += camZ;
}

Point BallHelperKinect::point3DToCam(int x, int y, int z) {
	x -= camX;
	y -= camY;
	z -= camZ;

	double dist = sqrt( x*x + y*y + z*z );
	double wDiff = y * virtualScreenDistance / (distPerPixel * dist) ;
	double hDiff = z * virtualScreenDistance / (distPerPixel * dist) ;

	Point p;
	p.x = wDiff + centerX;
	p.y = centerY - hDiff;
	return p;
}

void BallHelperKinect::doubleRes() {
	// Double Resolution back to original
	int row = FRAME_H - 1;
	for (int index_small = FRAME_PIX - 1; index_small >= 0; --index_small) {
		int index_b = 2 * index_small + row * 2*FRAME_W;

		m_cluster[index_b] = m_cluster[index_small];
		m_cluster[index_b + 1] = m_cluster[index_small];
		m_cluster[index_b + 2*FRAME_W] = m_cluster[index_small];
		m_cluster[index_b + 2*FRAME_W + 1] = m_cluster[index_small];

		if (col(index_small) == 0)
			--row;
	}
}

void BallHelperKinect::calculateRelations(KinectCluster* c, int n, int s,
		int w, int e) {
	c->north = n;
	c->south = s;
	c->west = w;
	c->east = e;

	c->height = c->south / FRAME_W - c->north / FRAME_W + 1;
	c->width = c->east % FRAME_W - c->west % FRAME_W + 1;

	c->invHeight = abs(c->east / FRAME_W - c->west / FRAME_W + 1);
	c->invWidth = abs(c->south % FRAME_W - c->north % FRAME_W + 1);
}

