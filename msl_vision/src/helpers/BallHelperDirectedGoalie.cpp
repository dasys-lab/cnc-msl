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
#include "BallHelperDirectedGoalie.h"
#include "SharedMemoryHelper.h"
#include "SharedBallDirectedHelper.h"
#include "RawOdometryHelper.h"
//#include <DateTime.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace std;

#include <sys/time.h>

#define MAX_BALL_BLOBS 500


//#define VERBOSE

BallHelperDirectedGoalie::BallHelperDirectedGoalie() : ballBuf(30){

	this->sc = SystemConfig::getInstance();

	FootballField::getInstance();

	//MX = atoi(vision->Values["CameraMX"].c_str());
	//MY = atoi(vision->Values["CameraMY"].c_str());
	//msgid = 1;	
	
	LocalizationSuccess = (*this->sc)["Localization"]->get<double>(
			"Localization", "LocalizationSuccess", NULL);

	ballProbs = (double *) malloc(MAX_BALL_BLOBS*sizeof(double));

	observations = (ObservedPoint *) malloc(10*sizeof(ObservedPoint));

	bzero(&currBlob, sizeof(BlobBounds));

	trackBlob.top = 0;
	trackBlob.bottom = 0;
	trackBlob.left = 0;
	trackBlob.right = 0;
	trackBlobConfidence = 0.0;

	velBlob.top = 200;
	velBlob.bottom = 280;
	velBlob.left = 280;
	velBlob.right = 360;

	velIterations = 1;

	blobVelX = 0.0;
	blobVelY = 0.0;

	int trackCounter = 0;

	init();


}


BallHelperDirectedGoalie::~BallHelperDirectedGoalie(){

	cleanup();

}



void BallHelperDirectedGoalie::init(){
	mv.point.x = 0.0;
	mv.point.y = 0.0;
	mv.velocity.vx = 0.0;
	mv.velocity.vy = 0.0;
}


void BallHelperDirectedGoalie::cleanup(){


}


Point BallHelperDirectedGoalie::getBallFromBlobs(ballCluster * cluster, int clusterCount, std::vector<ROIData>& roiData, std::vector<BlobBounds> & potBallBlobs){
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

/*
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

		p = posHelp->getPointCam2Point3DSony(cluster[i].x, cluster[i].y, (cluster[i].sizeSum / cluster[i].balls));

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

		ballPos.confidence = 0.4 + (((double)cluster[i].err/(double)cluster[i].balls)*0.2);
		if(irad>12) ballPos.confidence += 0.2;
		if(ballPos.confidence > 1.0) ballPos.confidence = 1.0;

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
*/	
/////////////////////////////

	printf("OldBall\n");
	ObservedPoint opOldBallDetection = getBallFromBlobsAdaptiveROI(cluster, clusterCount, roiData, potBallBlobs);
	if(opOldBallDetection.valid){
		printf("Stefan OldBall x: %f y: %f z: %f dist: %f\n", opOldBallDetection.x, opOldBallDetection.y, opOldBallDetection.z, sqrt(opOldBallDetection.x*opOldBallDetection.x + opOldBallDetection.y*opOldBallDetection.y));
		BallIntegrator::getInstance()->integratePoint(opOldBallDetection, 1000.0);	

		if(sharedMemoryIndex < 10){
			observations[sharedMemoryIndex] = opOldBallDetection;
			sharedMemoryIndex++;
			validBalls = true;
		}


	}

//////////////////////////////


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

	Point ballImPoint = posHelp->getPoint3D2CamSony(mv2.point.x, mv2.point.y, op.z);
	//cout << "Endy Elm2CamPos x:" << ballImPoint.x << " y:" << ballImPoint.y << endl;
	cout << "BallPos x:" << mv.point.x << " y:" << mv.point.y << " X-Vel: " << mv.velocity.vx << " Y-Vel: " << mv.velocity.vx <<endl;

	cout << "BallPos X-Vel: " << mv2.velocity.vx << "\tY-Vel: " << mv2.velocity.vx <<  endl;


	if(!(mv.point.x<0.1 && mv.point.x>-0.1 && mv.point.y<0.1 && mv.point.y>-0.1)) {
		ballROI.midX = ballImPoint.x;
		ballROI.midY = ballImPoint.y;
		int brad = posHelp->getPoint3D2RadiusSony(mv.point.x, mv.point.y, 0) + 10;
		ballROI.left = ballROI.midX - brad;
		ballROI.right = ballROI.midX + brad;
		ballROI.top = ballROI.midY - brad;
		ballROI.bottom = ballROI.midY + brad;
	
		roiData.push_back(ballROI);
	}

	//printf("BallTracker - BallContainer - startIndex = %d lastIndex = %d\n", ballBuf.getStartIndex(), ballBuf.getLastIndex());		
	//printf("BallTracker - MovingObject - %f %f %f %f\n", mv.point.x, mv.point.y, mv.velocity.vx, mv.velocity.vy);

	//EgoMotionEstimator * estimator = EgoMotionEstimator::getInstance();
	//MovingRobot mr = estimator->trackObject(rawHelper->getPositionBuffer(), rawHelper->getTimestampBuffer(), 10, rawHelper->getOdoIndex());

	//printf("EgoMotionEstimator - MovingRobot - %f %f %f\n", mr.velocity.vx, mr.velocity.vy, mr.velocity.w);
	return ballImPoint;
}

void BallHelperDirectedGoalie::visualizeBall(unsigned char * src, int width, Point ball, int radius){
	//cout << "Endy Endy2CamPos x:" << ball.x << " y:" << ball.y << endl;

	if(ball.x > 5.0 && ball.x < 474 && ball.y > 5.0 && ball.y < 634){

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

	if(currBlob.bottom > 0){

	       	for(int i = currBlob.left; i <= currBlob.right; i++){
	                src[currBlob.top*width + i] = 150;
        		src[currBlob.bottom*width + i] = 150;
	        }

        	for(int i = currBlob.top; i <= currBlob.bottom; i++){
	                src[i*width + currBlob.left] = 150;
        		src[i*width + currBlob.right] = 150;
	        }

	}

	if(trackBlob.bottom > 0){

		unsigned char color = 175;

		//if(trackBlobConfidence > 0.55)
		color = 60;

		BlobBounds trackBlobDisplay = trackBlob;
		if(trackBlob.top < 0)
			trackBlobDisplay.top = 0;
		if(trackBlob.left < 0)
			trackBlobDisplay.left = 0;
		if(trackBlob.right > 639)
			trackBlobDisplay.right = 639;
		if(trackBlob.bottom > 479)
			trackBlobDisplay.bottom = 479;

	       	for(int i = trackBlobDisplay.left; i <= trackBlobDisplay.right; i++){
	                src[trackBlobDisplay.top*width + i] = color;
        		src[trackBlobDisplay.bottom*width + i] = color;
	        }

        	for(int i = trackBlobDisplay.top; i <= trackBlobDisplay.bottom; i++){
	                src[i*width + trackBlobDisplay.left] = color;
        		src[i*width + trackBlobDisplay.right] = color;
	        }


	}


}


Point BallHelperDirectedGoalie::getBallPosition(){

	return currBallPos;

}


ObservedPoint BallHelperDirectedGoalie::getBallFromBlobsAdaptiveROI(ballCluster * cluster, int clusterCount, std::vector<ROIData>& roiData, std::vector<BlobBounds> & potBallBlobs){


	ObservedPoint op;

	bzero(&currBlob, sizeof(BlobBounds));

	PositionHelperDirected * posHelp = PositionHelperDirected::getInstance();

	CorrectedOdometry * pos = SharedMemoryHelper::getInstance()->readCorrectedOdometry();

	if(pos == NULL){
		printf("Pos = NULL!\n");
		return op;
	}

	printf("Corrected Odometry from VisionPlayer: %f %f %f %f\n", pos->posY, pos->posY, pos->posAngle, pos->posCertainty);


	op.timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();
	op.x = 0.0;
	op.y = 0.0;
	op.z = 0.0;
	op.angleHori = 0.0;
	op.angleVerti = 0.0;
	op.confidence = 0.0;
	op.valid = false;

	double savedBallProb = 0.0;
	bool inTrack = false;

	std::vector<BlobBounds> ballBlobs;

	for(int i = 0; i < MAX_BALL_BLOBS; i++){

		ballProbs[i] = 1.0;

	}



	if(potBallBlobs.size() > 0){


		double minBallDistance = 30000.0;
		unsigned int ballIndex = potBallBlobs.size() + 1;

		printf("Mops Tracker: %d\n", potBallBlobs.size());
	
//		printf("////////////////////////////////////////////////\n");
	
		for(unsigned int i = 0; i < potBallBlobs.size(); i++){
//#ifdef VERBOSE	
			printf("OldBall BallBlobWidth-%d: %d\n", i, potBallBlobs[i].right - potBallBlobs[i].left + 1);
			printf("OldBall BallBlobHeight-%d: %d\n", i, potBallBlobs[i].bottom - potBallBlobs[i].top + 1);
			printf("OldBall BallBlobMinDist-%d: %f\n", i, potBallBlobs[i].minDistance);
			printf("OldBall BallBlobCount-%d: %d\n", i, potBallBlobs[i].count);
//#endif 

			double diamX = fabs(potBallBlobs[i].bottom - potBallBlobs[i].top) + 1.0;
			double diamY = fabs(potBallBlobs[i].right - potBallBlobs[i].left) + 1.0;
			double diam = diamX;
			if(diam < diamY)
				diam = diamY;


			int midX = (potBallBlobs[i].top + potBallBlobs[i].bottom)/2;
			int midY = (potBallBlobs[i].left + potBallBlobs[i].right)/2;

			Point3D currPos = posHelp->getPointCam2Point3DSony(midX, midY, diam/2.0);

			potBallBlobs[i].minDistance = sqrt(currPos.x*currPos.x + currPos.y*currPos.y);	
			
			double currBallAngle = atan2(currPos.y, currPos.x);
	
			bool inField = false;


			if(pos == NULL || pos->posCertainty < LocalizationSuccess){
				inField = true;
			}
			else { 

	
				double alloBallPosX = pos->posX; //maxParticle->posx;
				double alloBallPosY = pos->posY; //maxParticle->posy;
	
				alloBallPosX += cos(pos->posAngle)*currPos.x - sin(pos->posAngle)*currPos.y;
				alloBallPosY += sin(pos->posAngle)*currPos.x + cos(pos->posAngle)*currPos.y;

				double relFactor = 500; //potBallBlobs[i].minDistance/4.0;
			/*	if(relFactor > 1500.0)
					relFactor = 1500.0;
				if(relFactor < 150.0)
					relFactor = 150.0;*/

				if(fabs(alloBallPosX) < FootballField::FieldLength/2.0 + relFactor && 
				 fabs(alloBallPosY) < FootballField::FieldWidth/2.0 + relFactor){
					inField = true;
				}

				if(alloBallPosX < - FootballField::FieldLength - 100.0)
					inField = false;

			}



			if(inField){
	
				if(potBallBlobs[i].minDistance < minBallDistance && 
					potBallBlobs[i].minDistance > 100.0 &&
					potBallBlobs[i].count > 20) {
					minBallDistance = potBallBlobs[i].minDistance;
					ballIndex = i;
				}

			}
	
//			printf("////////////////////////////////////////////////\n\n");	


			if(potBallBlobs[i].minDistance < 100.0 || potBallBlobs[i].count < 15)
				ballProbs[i] = 0.0;
			if(!inField)
				ballProbs[i] = 0.0;
			
			
			double blobHeight = (potBallBlobs[i].bottom - potBallBlobs[i].top + 1);
			double blobWidth = (potBallBlobs[i].right - potBallBlobs[i].left + 1);
			double ratio = blobHeight/blobWidth;
			if(ratio > 1.0)
				ratio = 1.0/ratio;

			ballProbs[i] *= sqrt(sqrt(ratio));


			double coverage = potBallBlobs[i].count/(blobHeight*blobWidth);
			ballProbs[i] *= sqrt(coverage);			

			double expectedCount = 1.0; //diam*diam*0.8;
			if(expectedCount > 1.0)
				expectedCount = 1.0/expectedCount;
			ballProbs[i] *= sqrt(sqrt(expectedCount));

			if(currPos.z < 120.0)
				currPos.z = 120.0;


			if(currPos.z >= 550.0)
				ballProbs[i] -= currPos.z/5000.0;


			if(ballProbs[i] > 0.5)
				ballProbs[i] = (ballProbs[i] - 0.4)*0.5 + 0.4;


			if(ballProbs[i] > 0.9)
				ballProbs[i] = 0.9;


			int trackBlobMidX = (trackBlob.top + trackBlob.bottom)/2;
			int trackBlobMidY = (trackBlob.left + trackBlob.right)/2;


double distTrack = sqrt(3*(trackBlobMidX - midX)*(trackBlobMidX - midX) + (trackBlobMidY - midY)*(trackBlobMidY - midY));


			printf("Mops Tracker distTrack: %f \n", distTrack);

			printf("Mops Tracker Stefan: %f\n", ballProbs[i]);


			double krautX = (trackBlobMidX - midX);
			double krautY = (trackBlobMidY - midY);

			printf("Mops Tracker Stefan: %f %f \n", krautX, krautY);


			if(ballProbs[i] > 0.22 && distTrack < 100.0){

				inTrack = true;
				savedBallProb = ballProbs[i];
				ballProbs[i] = trackBlobConfidence + (100 - distTrack)/10000.0;
				//ballProbs[i] = trackBlobConfidence;

				printf("Mops Tracker Stefan in Track\n");
			} else if(currPos.z > 550.0)
				ballProbs[i] = 0.0;
	

	
		}
	
		printf("Mops Tracker Stefan savedBallProb: %f\n");

		unsigned int maxProbIndex = potBallBlobs.size();

		minBallDistance = 30000.0;
		ballIndex = potBallBlobs.size();
		double maxProb = 0.0;
		double realBallProb = 0.0;
		for(unsigned int i = 0; i < potBallBlobs.size(); i++){
			if(ballProbs[i] > maxProb){
				maxProbIndex = i;
				ballIndex = i;
				maxProb = ballProbs[i];
				realBallProb = ballProbs[i];
			}

		}


		if(ballIndex < potBallBlobs.size()){	
	
			ballBlobs.clear();
			ballBlobs.push_back(potBallBlobs[ballIndex]);
			
	//		printf("+++++potBallBlobs.size: %d\n", potBallBlobs.size());				
		
			double diamX = fabs(ballBlobs[0].bottom - ballBlobs[0].top) + 1.0;
			double diamY = fabs(ballBlobs[0].right - ballBlobs[0].left) + 1.0;
			double diam = diamX;
			if(diam < diamY)
				diam = diamY;


			int midX = (ballBlobs[0].top + ballBlobs[0].bottom)/2;
			int midY = (ballBlobs[0].left + ballBlobs[0].right)/2;


			Point3D pOld = posHelp->getPointCam2Point3DSony(midX, midY, diam/2.0);		

			double diamAdd = 0.0;
			if(pOld.z > 200.0)
				diamAdd = 2.0*pOld.z/150.0;

			if(diamAdd > 10.0)
				diamAdd = 10.0;

			diam += diamAdd;
			Point3D p = posHelp->getPointCam2Point3DSony(midX, midY, diam/2.0);

			if(sqrt(p.x*p.x + p.y*p.y) > 3000.0 && p.z < 300.0){

				Point p0 = posHelp->getPointCam2FieldSony(midX, ballBlobs[0].right);
				p.x = p0.x;
				p.y = p0.y;
				//if(p.z < 120)
				//	p.z = 120.0;

			}

			currBlob = ballBlobs[0];

			if(savedBallProb > 0.35){
				trackBlobConfidence += 0.1; //0.05
				if(trackBlobConfidence > 0.9)
					trackBlobConfidence = 0.9;
			}
			if(savedBallProb < 0.25){

				trackBlobConfidence -= 0.075; //0.03
				if(trackBlobConfidence < 0.30){
					trackBlobConfidence = 0.30;
					inTrack = false;
					trackBlob.top = 0;
					trackBlob.bottom = 0;
					trackBlob.left = 0;
					trackBlob.right = 0;
					trackBlobConfidence = 0;
				}	

			}

			printf("Stefan RealBallProb: %f SavedBallProb: %f TrackBlobConfidence %f inTrack: %d\n", realBallProb, savedBallProb, trackBlobConfidence, inTrack);

			if(p.z < 350 && realBallProb > 0.33 || inTrack){
				trackBlob = ballBlobs[0];
				if(!inTrack){
					trackBlobConfidence = realBallProb;
					trackCounter = 15;
					inTrack = true;
				}
				//inTrack = true;
			}

			op.x = p.x; //cos(ballangle)*ballBlobs[0].minDistance;
			op.y = p.y; //sin(ballangle)*ballBlobs[0].minDistance;
			op.z = p.z;
			if(inTrack)
				op.confidence = savedBallProb;
			else
				op.confidence = realBallProb;

			op.confidence -= 0.1;

			if(trackBlobConfidence > 0.0 || p.z < 350)
				op.valid = true;
			else
				op.valid = false;

			if(inTrack && p.z > 350.0)
				trackCounter--;

			if(trackCounter <= 0){
				trackBlob.top = 0;
				trackBlob.bottom = 0;
				trackBlob.left = 0;
				trackBlob.right = 0;
				trackBlobConfidence = 0;
				inTrack = false;
			}				

			if(!inTrack){
				trackBlobConfidence = 0.0;
				trackCounter = 0;
			}

			if(inTrack && op.confidence < 0.5)
				op.confidence = 0.5;

			printf("Mops Tracker: px: %f py: %f pz: %f conf: %f track: %f intrack: %d real: %f\n", op.x, op.y, op.z, op.confidence, trackBlobConfidence, inTrack, realBallProb);

			return op;

		}
		else {

			return op;


		}
	}
	else{

		return op;

	}

}


