/*
 * $Id: BallHelperDirected.cpp 1935 2007-03-19 19:50:12Z phbaer $
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
#include "BallHelperDirected2.h"
#include "SharedMemoryHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace std;

#include <sys/time.h>

#define MAX_BALL_BLOBS 500


//#define VERBOSE


BallHelperDirected2::BallHelperDirected2() : ballBuf(30){

	this->sc = SystemConfig::getInstance();

	FootballField::getInstance();

	//MX = atoi(vision->Values["CameraMX"].c_str());
	//MY = atoi(vision->Values["CameraMY"].c_str());
	//msgid = 1;	
	
	LocalizationSuccess = (*this->sc)["Localization"]->get<double>(
			"Localization", "LocalizationSuccess", NULL);

	ballProbs = (double *) malloc(MAX_BALL_BLOBS*sizeof(double));

	observations = (ObservedPoint *) malloc(10*sizeof(ObservedPoint));

	width=640;
	height=480;

	init();


}


BallHelperDirected2::~BallHelperDirected2(){

	cleanup();

}



void BallHelperDirected2::init(){
	mv.point.x = 0.0;
	mv.point.y = 0.0;
	mv.velocity.vx = 0.0;
	mv.velocity.vy = 0.0;
}


void BallHelperDirected2::cleanup(){


}


Point BallHelperDirected2::getBallFromBlobs(ballCluster * cluster, int clusterCount, std::vector<ROIData>& roiData){
	ObservedPoint ballPos;
	ROIData ballROI;
	//BlobBounds retBlob;
	bool validBalls = false;

	PositionHelperDirected * posHelp = PositionHelperDirected::getInstance();

	ballPos.x = 0.0;
	ballPos.y = 0.0;
	ballPos.z = 0.0;
	ballPos.confidence = 0.0;
	ballPos.valid = 0.0;

	Point3D p;



	int sharedMemoryIndex = 0;


	for(int i=0; i<clusterCount; i++) {
		int pos = i;
		int irad = (cluster[i].sizeSum / cluster[i].balls);
		bool valid=true;

		//Ignore Balls which are in other balls
		for(int n=0; n<clusterCount; n++) {
			if(n==i) continue;
			int nrad = cluster[n].sizeSum / cluster[n].balls;
			int xdist = cluster[i].x - cluster[n].x;
			int ydist = cluster[i].y - cluster[n].y;

			if(xdist < 0) xdist = -xdist;
			if(ydist < 0) ydist = -ydist;

			if(nrad>irad && (xdist*ydist < nrad*nrad)) {
				valid=false;
			}
		}
		if(!valid)
			continue;

		p = posHelp->getPointCam2Point3D(cluster[i].x, cluster[i].y, (cluster[i].sizeSum / cluster[i].balls));

		ballPos.x = -p.x;
		ballPos.y = p.y;
		ballPos.z = p.z;
		ballPos.valid = true;

		cout << "Endy Z-Pos " << ballPos.z << " X: " << ballPos.x << " Y: " << ballPos.y << " Dist: "<< sqrt(ballPos.x*ballPos.x + ballPos.y+ballPos.y) << " Rad " << (cluster[i].sizeSum / cluster[i].balls) << endl;
		double relFactor = 2000;
		if(fabs(p.x) > FootballField::FieldLength + relFactor || 
			fabs(p.y) > FootballField::FieldWidth/2 + relFactor) {
			continue;
		}

		if(p.z < -300) continue;

		ballPos.confidence = 0.35 + (((double)cluster[i].err/(double)cluster[i].balls)*0.2);
		if(irad>12) ballPos.confidence += 0.2;
		if(p.z > 500) ballPos.confidence *= 0.7;
		if(ballPos.confidence > 0.9) ballPos.confidence = 0.9;

		printf("Endy x: %d; y: %d; radius: %d\n", cluster[i].x, cluster[i].y, cluster[i].sizeSum / cluster[i].balls);
		cout << "Endy ElmPos x:" << p.x << " y:" << p.y << " z: " << p.z << endl;

		ballPos.timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();
	
		//printf("BallTracker - BallHelper - ballPos %f %f %d %ld\n", ballPos.x, ballPos.y, ballPos.valid, ballPos.timestamp);
		//printf("getLastIndex : %d\n", ballBuf.getLastIndex());	
	
		BallIntegrator::getInstance()->integratePoint(ballPos, 1000.0);
		validBalls = true;
		//printf("getLastIndex : %d\n", ballBuf.getLastIndex());

		if(sharedMemoryIndex < 10){

			observations[sharedMemoryIndex] = ballPos;
			sharedMemoryIndex++;

		}


	}

	if(!validBalls) {
		ballPos.x = 0.0;
        	ballPos.y = 0.0;
	        ballPos.z = 0.0;
        	ballPos.confidence = 0.0;
	        ballPos.valid = false;

		BallIntegrator::getInstance()->integratePoint(ballPos, 1000.0);
		cout << "BallPos - no Ball" << endl;
	}

	BallIntegrator::getInstance()->decreaseDirtyPointCertainty();
//////////////////////////////////////////////////

	for(int i = sharedMemoryIndex; i < 10; i++){

		observations[i].valid = false;

	}

	SharedMemoryHelper::getInstance()->writeDirectedBallPosition(observations);

	ObservedPoint op = BallIntegrator::getInstance()->getPoint();
	op.timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();

	double dist = sqrt( (op.x - mv.point.x)*(op.x - mv.point.x) + (op.y - mv.point.y)*(op.y - mv.point.y) );

	if(dist > 750.0)
		ballBuf.reset(); 

	ballBuf.integratePoint(op);
	printf("BallTracker Validity: %d\n");
	//printf("Before invalidate\n");
	ballBuf.invalidate(400);

	mv = ObjectTracker::getInstance()->trackObject(ballBuf.getPoints(), ballBuf.getSize(), ballBuf.getStartIndex(), ballBuf.getLastIndex(), 0.3E07);
	RawOdometryHelper * rawHelper = RawOdometryHelper::getInstance();

	MovingObject mv2 = mv;

	mv2.point = rawHelper->allo2EgoOnVision(mv.point);
	mv2.velocity = rawHelper->allo2Ego(mv.velocity, rawHelper->getVisionPos());

	Point ballImPoint = posHelp->getPoint3D2Cam(mv2.point.x, mv2.point.y, op.z);
	if(ballImPoint.x < 17) ballImPoint.x=17;
	if(ballImPoint.x > width-17) ballImPoint.x=width-17;
	if(ballImPoint.y < 17) ballImPoint.y=17;
	if(ballImPoint.y > height-17) ballImPoint.y=height-17;

	cout << "Endy Elm2CamPos x:" << ballImPoint.x << " y:" << ballImPoint.y << endl;
	cout << "BallPos x:" << mv.point.x << " y:" << mv.point.y << " X-Vel: " << mv.velocity.vx << " Y-Vel: " << mv.velocity.vx <<endl;

	cout << "BallPos X-Vel: " << mv2.velocity.vx << "\tY-Vel: " << mv2.velocity.vx <<  endl;


	if(!(mv.point.x<0.1 && mv.point.x>-0.1 && mv.point.y<0.1 && mv.point.y>-0.1)) {
		ballROI.midX = ballImPoint.x;
		ballROI.midY = ballImPoint.y;
		int brad=PositionHelperDirected::getInstance()->getPoint3D2Radius(mv.point.x, mv.point.y, 0) + 10;
		ballROI.left = ballROI.midX - brad;
		ballROI.right = ballROI.midX + brad;
		ballROI.top = ballROI.midY - brad;
		ballROI.bottom = ballROI.midY + brad;

		if(ballROI.midX<0) ballROI.midX=0;
                if(ballROI.midY<0) ballROI.midY=0;
                if(ballROI.left<0) ballROI.left=0;
                if(ballROI.top<0) ballROI.top=0;
                if(ballROI.right<0) ballROI.right=0;
                if(ballROI.bottom<0) ballROI.bottom=0;

                if(ballROI.midX>width) ballROI.midX=width;
                if(ballROI.midY>height) ballROI.midY=height;
                if(ballROI.left>width) ballROI.left=width;
                if(ballROI.top>height) ballROI.top=height;
                if(ballROI.right>width) ballROI.right=width;
                if(ballROI.bottom>height) ballROI.bottom=height;
	

		roiData.push_back(ballROI);
	}

	//printf("BallTracker - BallContainer - startIndex = %d lastIndex = %d\n", ballBuf.getStartIndex(), ballBuf.getLastIndex());		
	//printf("BallTracker - MovingObject - %f %f %f %f\n", mv.point.x, mv.point.y, mv.velocity.vx, mv.velocity.vy);

	//EgoMotionEstimator * estimator = EgoMotionEstimator::getInstance();
	//MovingRobot mr = estimator->trackObject(rawHelper->getPositionBuffer(), rawHelper->getTimestampBuffer(), 10, rawHelper->getOdoIndex());

	//printf("EgoMotionEstimator - MovingRobot - %f %f %f\n", mr.velocity.vx, mr.velocity.vy, mr.velocity.w);
	return ballImPoint;
}

void BallHelperDirected2::visualizeBall(unsigned char * src, int width, Point ball, int radius){
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


Point BallHelperDirected2::getBallPosition(){

	return currBallPos;

}


