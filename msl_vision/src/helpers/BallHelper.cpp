/*
 * $Id: BallHelper.cpp 1935 2007-03-19 19:50:12Z phbaer $
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
#include "BallHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>

using namespace std;

#include <sys/time.h>

#include "FootballField.h"
#include <msl_sensor_msgs/BallInfo.h>
//#include <BallInfoCovMatrix.h> //merge to dev 
#include "SpicaHelper.h"
#include "BallIntegrator.h"
#include "ObjectTracker.h"
#include "BallZTracker.h"
#include "TimeHelper.h"
#include "EgoMotionEstimator.h"
#include "SharedMemoryHelper.h"
#include "CovarianceHelper.h"
#include <ros/callback_queue.h>
#include <ros/transport_hints.h>
#include "msl_sensor_msgs/BallHypothesis.h"

#define MAX_BALL_BLOBS 500


//#define VERBOSE
using namespace msl_sensor_msgs;
using namespace msl_msgs;

BallHelper::BallHelper(int area) : ballBuf(30){

	this->sc = SystemConfig::getInstance();

	FootballField::getInstance();

	//MX = atoi(vision->Values["CameraMX"].c_str());
	//MY = atoi(vision->Values["CameraMY"].c_str());
	//msgid = 1;	
	MX = area/2;//atoi(vision->Values["CameraMX"].c_str());
	MY = area/2;//atoi(vision->Values["CameraMY"].c_str());

	//HACK Dom New Optic 
	newOptics = false;
	newOptics = (*this->sc)["Vision"]->tryGet<bool>(false,
			"Vision", "NewOptics", NULL);
	
	LocalizationSuccess = (*this->sc)["Localization"]->get<double>(
			"Localization", "LocalizationSuccess", NULL);

	ballProbs = (double *) malloc(MAX_BALL_BLOBS*sizeof(double));

	currBallPos.x = 0.0;
	currBallPos.y = 0.0;

	ballProbs = (double *) malloc(MAX_BALL_BLOBS*sizeof(double));

	isGoalie = (*this->sc)["Globals"]->get<bool>("Globals", "Team", SystemConfig::getHostname().c_str(), "IsGoalie", NULL);
	passMsgAvailable = false;

	init();
	


	std::cout << "Add callback" << std::endl;
	sub = SpicaHelper::visionNode->subscribe<PassMsg, BallHelper>("PassMsg", 1, &BallHelper::handlePassMessageInfo, (this), ros::TransportHints().udp());
	std::cout << "Add callback done" << std::endl;


}


BallHelper::~BallHelper(){

	cleanup();

}



void BallHelper::handlePassMessageInfo(const PassMsg::ConstPtr& message) {
	this->origin = message->origin;
	this->destination = message->destination;
	gettimeofday(&passMsgTime, NULL);
	passMsgAvailable = true;
	cout << "PassBall Message received" << endl;
}





void BallHelper::init(){

	SharedMemoryHelper::getInstance();

	mv.point.x = 0.0;
	mv.point.y = 0.0;
	mv.velocity.vx = 0.0;
	mv.velocity.vy = 0.0;
}


void BallHelper::cleanup(){


}


void BallHelper::sendBallHypotesis(ballCluster * cluster, int clusterCount, std::vector<ROIData>& roiData){
	static int noBallCycles = 0;
	bool ballIntegrated = false;

	ROIData ballROI;
	PositionHelper * posHelp = PositionHelper::getInstance();
	msl_sensor_msgs::BallHypothesis ballPos;
	ballPos.egoPosition.x = 0.0;
	ballPos.egoPosition.y = 0.0;
	ballPos.egoPosition.z = 0.0;
	ballPos.confidence = 0.0;
	Point3D p;

	SpicaHelper::ballList->hypothesis.clear();
	SpicaHelper::ballList->imageTime = TimeHelper::getInstance()->getVisionTimeOmniCam();

	printf("Ballhelper: Cluster Count %d\n", clusterCount);
	for(int i=0; i<clusterCount; i++) {
		int pos = i;
		int irad = (cluster[i].sizeSum / cluster[i].balls);
		ballPos.radius = irad;
		ballPos.cameraCoordinates.x = cluster[i].x;
		ballPos.cameraCoordinates.y = cluster[i].y;
		ballPos.detectedNearbyCircles = cluster[i].balls;

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

		//Only use the hypothesis which is close to the ground
		int grounddist = 100000;
		for(int r=cluster[i].minRadius; r<=cluster[i].maxRadius; r++) {
			if(r+cluster[i].x >= 460 || r+cluster[i].y >= 460) {
				valid = false;
			}
			else {
				p = posHelp->getPointCam2Point3D(cluster[i].y, cluster[i].x, r);

				if(fabs(p.z-130) < grounddist) {
					ballPos.egoPosition.x = p.x;
					ballPos.egoPosition.y = p.y;
					ballPos.egoPosition.z = p.z;

					valid = true;
					grounddist = (int)fabs(p.z-130);
				}
			}
		}
		if(!valid)
			continue;

		cout << "BallHypothesis: Z-Pos " << p.z << " X: " << p.x << " Y: " << p.y << " FL: " << FootballField::FieldLength << " FW: " << FootballField::FieldWidth << endl;
		double relFactor = 200;
		if(fabs(p.x) > FootballField::FieldLength + relFactor ||
			fabs(p.y) > FootballField::FieldWidth + relFactor) {
			continue;
		}


		//achtung!!! passball hier evtl rausnehmen?
		if(p.z < -300) {
			continue;
		}

		if(p.z > 350) continue;
		if(p.x*p.x+p.y*p.y>8000*8000) continue;
		if(isGoalie && p.x*p.x+p.y*p.y>5500*5500) continue;

		ballPos.confidence = 0.3 + (((double)cluster[i].err/(double)cluster[i].balls)*0.1);
		if(irad>5) ballPos.confidence += 0.2;
		if(p.x*p.x+p.y*p.y < 1000*1000) ballPos.confidence = 0.8;

		//Mapping Position on the Ground
		if(p.z > 350) {
			ballPos.confidence *= 0.9;

		} else {
			p = posHelp->getBallPositionFromBallMid(cluster[i].y, cluster[i].x);

			ballPos.egoPosition.x = p.x;
			ballPos.egoPosition.y = p.y;
			ballPos.egoPosition.z = p.z;
		}
		if(ballPos.confidence > 0.9) ballPos.confidence = 0.9;
		ballPos.errors=(double)cluster[i].err/(double)cluster[i].balls;

		SpicaHelper::ballList->hypothesis.push_back(ballPos);

	}
	SpicaHelper::sendBallHypothesis();
}

Point BallHelper::getBallFromBlobs(ballCluster * cluster, int clusterCount, std::vector<ROIData>& roiData, Particle * maxParticle){
	static int noBallCycles = 0;
	bool ballIntegrated = false;

	struct timeval curTime;
	gettimeofday(&curTime, NULL);
	long timediff;
	if(passMsgAvailable) {
		timediff = curTime.tv_sec - passMsgTime.tv_sec;
		if(timediff < 0)
			timediff += 1000000;
		if(timediff >= 4) {
			passMsgAvailable = false;
			cout << "PassBall Message timed out" << endl;
		}
	}

	ObservedPoint ballPos;
	ROIData ballROI; 

	// Integrate balls from directed camera
	ObservedPoint * opDirected = SharedMemoryHelper::getInstance()->readDirectedBallPosition();

	for(int i = 0; i < 10; i++){
		if(opDirected[i].valid){
			BallIntegrator::getInstance()->integratePoint(opDirected[i], 1000.0);	
			printf("BallPos Integrate Directed: %f %f %f %lld\n", opDirected[i].x, opDirected[i].y, opDirected[i].z, opDirected[i].timestamp);
		}
	}

	// Integrate balls from Kinect
	opDirected = SharedMemoryHelper::getInstance()->readKinectBallPosition();

	for(int i = 0; i < 10; i++){
		if(opDirected[i].valid){
			BallIntegrator::getInstance()->integratePoint(opDirected[i], 1000.0);
			printf("BallPos Integrate Directed: %f %f %f %lld\n", opDirected[i].x, opDirected[i].y, opDirected[i].z, opDirected[i].timestamp);
		}
	}

	PositionHelper * posHelp = PositionHelper::getInstance();

	ballPos.x = 0.0;
	ballPos.y = 0.0;
	ballPos.z = 0.0;
	ballPos.confidence = 0.0;
	ballPos.valid = false;

	Point3D p;

	printf("Ballhelper: Cluster Count %d\n", clusterCount);

	if(clusterCount==0)BallIntegrator::getInstance()->integratePoint(ballPos, 1000.0);

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

		int grounddist = 100000; 
		for(int r=cluster[i].minRadius; r<=cluster[i].maxRadius; r++) {
			if(r+cluster[i].x >= 460 || r+cluster[i].y >= 460) {
				ballPos.valid = false;
			}
			else {
				p = posHelp->getPointCam2Point3D(cluster[i].y, cluster[i].x, r);

				if(fabs(p.z-130) < grounddist) {
					ballPos.x = p.x;
					ballPos.y = p.y;
					ballPos.z = p.z;
	
					ballPos.valid = true;
					grounddist = (int)fabs(p.z-130);
				}
			}
		}

		cout << "Ballintegrator BallHypothesis: Z-Pos " << p.z << " X: " << p.x << " Y: " << p.y << " FL: " << FootballField::FieldLength << " FW: " << FootballField::FieldWidth << endl;
		double relFactor = 200;
		if(fabs(p.x) > FootballField::FieldLength + relFactor || 
			fabs(p.y) > FootballField::FieldWidth + relFactor) {
			continue;
		}

		bool inField = false;
		bool passBall = false;

		if(maxParticle == NULL || maxParticle->weight < LocalizationSuccess){
			inField = true;
		}
		else { 
	
			double alloBallPosX = maxParticle->posx;
			double alloBallPosY = maxParticle->posy;
	
			alloBallPosX += cos(maxParticle->heading)*p.x - sin(maxParticle->heading)*p.y;
			alloBallPosY += sin(maxParticle->heading)*p.x + cos(maxParticle->heading)*p.y;

			/*	if(relFactor > 1500.0)
					relFactor = 1500.0;
				if(relFactor < 150.0)
					relFactor = 150.0;*/

			if(fabs(alloBallPosX) < FootballField::FieldLength/2.0 + relFactor && 
			 fabs(alloBallPosY) < FootballField::FieldWidth/2.0 + relFactor){
				inField = true;
			}
			if(passMsgAvailable) {
				double a;
				double b=-1;
				double c;
				double m=alloBallPosX;
				double n=alloBallPosY;
				double passDist;

				if(destination.x - origin.x != 0) {
					a = (destination.y - origin.y) / (destination.x - origin.x);
					c = -destination.x*a + destination.y;
				
					passDist = fabs(a*m+b*n+c)/sqrt(a*a+b*b);
				}
				else passDist = fabs(destination.x-alloBallPosX);

				double eineVar = ( origin.y - destination.y);
				double zweiteVar = ( origin.x - destination.x);
				double passLength = sqrt(eineVar*eineVar+zweiteVar*zweiteVar);

				eineVar = (origin.y - alloBallPosY);
				zweiteVar = (origin.x - alloBallPosX);
				double ballHypDist = sqrt(eineVar*eineVar+zweiteVar*zweiteVar);

				if(passDist < 1500 && passLength>ballHypDist) {
					passBall = true;
					cout << "PassBall detectet at:" << alloBallPosX <<" " << alloBallPosY << " Z: "  << p.z << endl;
				}
			}

		}

		if(!inField) continue;

		//achtung!!! passball hier evtl rausnehmen?
		//if(p.z < -300 && !passBall) continue;
		if(p.z < -300) {
			continue;
		}

		if(p.z > 350 && !passBall) continue;
		//if(p.z > 350 && p.x*p.x+p.y*p.y>750*750) {
		//	continue;
		//}
		if(p.x*p.x+p.y*p.y>8000*8000) continue;
		if(isGoalie && p.x*p.x+p.y*p.y>5500*5500) continue;

		ballPos.confidence = 0.3 + (((double)cluster[i].err/(double)cluster[i].balls)*0.1);
		if(irad>5) ballPos.confidence += 0.2;
		if(p.x*p.x+p.y*p.y < 1000*1000) ballPos.confidence = 0.8;
		if(p.z > 350) {
			ballPos.confidence *= 0.9;

		} else {
			p = posHelp->getBallPositionFromBallMid(cluster[i].y, cluster[i].x);

			ballPos.x = p.x;
			ballPos.y = p.y;
			ballPos.z = p.z;
		}
		if(ballPos.confidence > 0.9) ballPos.confidence = 0.9;

		ballPos.timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();
	
		BallIntegrator::getInstance()->integratePoint(ballPos, 1000.0);
		ballIntegrated = true;
		noBallCycles=0;
	}
	if(!ballIntegrated) noBallCycles++;
////////////////////////////////////here old ball helper////////////////////////////////////
#ifdef OLDBALL
	if(noBallCycles>3 && !passMsgAvailable) {
		ObservedPoint opOldBallDetection = getBallFromBlobsAdaptiveROI(cluster, clusterCount, roiData, potBallBlobs, maxParticle);
		if(opOldBallDetection.valid){
			printf("OldBall x: %f y: %f z: %f dist: %f\n", opOldBallDetection.x, opOldBallDetection.y, opOldBallDetection.z, sqrt(opOldBallDetection.x*opOldBallDetection.x + opOldBallDetection.y*opOldBallDetection.y));
			BallIntegrator::getInstance()->integratePoint(opOldBallDetection, 1000.0);	
		}
	}
#endif

////////////////////////////////////here old ball helper finsihed//////////////////////////


	BallIntegrator::getInstance()->decreaseDirtyPointCertainty();
//////////////////////////////////////////////////



	ObservedPoint op = BallIntegrator::getInstance()->getPoint();
	op.timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();

	ObjectContainer * currBallBuf = BallIntegrator::getInstance()->getContainer();

	if(currBallBuf == NULL){
		currBallBuf = &ballBuf;
		printf("BallPos CurrBallBuf is NULL\n");
	}

	currBallBuf->invalidate(400);

	double dist = sqrt( (op.x - mv.point.x)*(op.x - mv.point.x) + (op.y - mv.point.y)*(op.y - mv.point.y) );

	//if(dist > 750.0){
	//	currBallBuf->reset();
	//	printf("CurrBallBuf RESET\n"); 
	//}
	//ballBuf.integratePoint(op);
	//printf("BallTracker Validity: %d\n");
	//printf("Before invalidate\n");
	//ballBuf.invalidate(400);


	mv = ObjectTracker::getInstance()->trackObject(currBallBuf->getPoints(), currBallBuf->getSize(), currBallBuf->getStartIndex(), currBallBuf->getLastIndex(), 0.3E07);

	bool noValidPoint = false;

	if(fabs(mv.point.x) > 50000.0 && fabs(mv.point.y) > 50000.0)
		noValidPoint = true;


	ZEstimate ze = BallZTracker::getInstance()->trackObject(currBallBuf->getPoints(), currBallBuf->getSize(), currBallBuf->getStartIndex(), currBallBuf->getLastIndex());

	RawOdometryHelper * rawHelper = RawOdometryHelper::getInstance();

	MovingObject mv2 = mv;
	MovingObject opMV = mv;
	opMV.point.x = op.x;
	opMV.point.y = op.y;

//	mv2.point = rawHelper->allo2EgoOnVision(opMV.point);
//	mv2.velocity = rawHelper->allo2Ego(mv.velocity, rawHelper->getVisionPos());


//	mv2.point = rawHelper->allo2EgoOnVision(mv.point);
//	mv2.velocity = rawHelper->allo2Ego(mv.velocity, rawHelper->getVisionPos());

	
	mv2.point = rawHelper->allo2Ego(mv.point, BallIntegrator::getInstance()->getRefPosition());
	mv2.velocity = rawHelper->allo2Ego(mv.velocity, BallIntegrator::getInstance()->getRefPosition());



	Point ballImPoint = posHelp->getPoint3D2Cam(mv2.point.x, mv2.point.y, op.z);
	//cout << "Endy Elm2CamPos x:" << ballImPoint.x << " y:" << ballImPoint.y << endl;
	cout << "BallPos + OldBall x:" << mv.point.x << " y:" << mv.point.y << " X-Vel: " << mv.velocity.vx << " Y-Vel: " << mv.velocity.vy <<endl;

	if(!(mv.point.x<0.1 && mv.point.x>-0.1 && mv.point.y<0.1 && mv.point.y>-0.1)) {
		ballROI.midX = ballImPoint.y;
		ballROI.midY = ballImPoint.x;
		ballROI.left = ballROI.midX - 17;
		ballROI.right = ballROI.midX + 17;
		ballROI.top = ballROI.midY - 17;
		ballROI.bottom = ballROI.midY + 17;
	
		roiData.push_back(ballROI);
	}

	//cout << "BalPos X-Vel: " << mv.velocity.vx << "\tY-Vel: " << mv.velocity.vx <<endl;



	//printf("BallTracker - BallContainer - startIndex = %d lastIndex = %d\n", ballBuf.getStartIndex(), ballBuf.getLastIndex());		
	//printf("BallTracker - MovingObject - %f %f %f %f\n", mv.point.x, mv.point.y, mv.velocity.vx, mv.velocity.vy);

	//EgoMotionEstimator * estimator = EgoMotionEstimator::getInstance();
	//MovingRobot mr = estimator->trackObject(rawHelper->getPositionBuffer(), rawHelper->getTimestampBuffer(), 10, rawHelper->getOdoIndex());

	//printf("EgoMotionEstimator - MovingRobot - %f %f %f\n", mr.velocity.vx, mr.velocity.vy, mr.velocity.w);


//	ObservedPoint directedBall = SharedMemoryHelper::getInstance()->readBallPosition();


	Point3dInfo ballPoint;// = Point3dInfo::create();
	Velocity3dInfo ballVelocity;// = Velocity3dInfo::create();
	//not any more in dev branch dki
	//CovMatrix3DPtr covMatrix = CovMatrix3D::create();
	
	//boost::shared_ptr<std::vector<double> > covariances = boost::shared_ptr<std::vector<double> >(new std::vector<double>);
	
	
	if(op.valid && !noValidPoint){
		printf("BallPos sent: %f %f %f\n", mv2.point.x, mv2.point.y, ze.z);
		ballPoint.x = (mv2.point.x);
		ballPoint.y = (mv2.point.y);
		ballPoint.z = (ze.z);
		ballVelocity.vx = (mv2.velocity.vx);
		ballVelocity.vy = (mv2.velocity.vy);
		ballVelocity.vz = (ze.vz);

		double mean[3];
			
		mean[0] = atan2(mv2.point.y, mv2.point.x);
		mean[1] = sqrt(mv2.point.x*mv2.point.x + mv2.point.y*mv2.point.y);
		mean[2] = ze.z;
			
		double currDistStd = 25.0 + mean[1]/10.0;
		double currAngleStd = 8.0 - mean[1]/1000.0;
		if(currAngleStd < 1.0)
			currAngleStd = 1.0;
			
		currAngleStd = currAngleStd*180.0/M_PI;
			
			
		double cov[9];			
		cov[0] = currAngleStd*currAngleStd;
		cov[4] = currDistStd*currDistStd;
		cov[8] = 300.0*300.0;
			
		double meanRes[9];
		double covRes[9];
			
		CovarianceHelper::TransformCovMatrixPolar2Ego(mean, cov, meanRes, covRes, 3);
			
		//merge to dev dki
		//for(int i = 0; i < 9; i++){
		//	covariances->push_back(covRes[i]);
		//}
		
		//covMatrix->setValues(covariances);
		
		
	}
	else {

		ballPoint.x = (0.0);
		ballPoint.y = (0.0);
		ballPoint.z = (0.0);

		ballVelocity.vx = (0.0);
		ballVelocity.vy = (0.0);
		ballVelocity.vz = (0.0);
			
		//merge to dev
		//for(int i = 0; i < 9; i++){
		//	covariances->push_back(0.0);
		//}
		
		//covMatrix->setValues(covariances);

	}
		
		

		//merge to dev
		//BallInfoCovMatrixPtr bi = BallInfoCovMatrix::create();


	BallInfo bi;// = BallInfo::create();
	if(mv2.point.x == 0.0 && mv2.point.y==0.0) op.confidence = 0.0;
		
	bi.point = (ballPoint);
	bi.velocity = (ballVelocity);
	bi.confidence = (op.confidence);
	bi.ballType = (1);
	//	bi->setCovarianceMatrix(covMatrix);  //merge to dev

	printf("Check Confidence: %f\n", op.confidence);

	printf("StefansKraut %f %f %f %f\n", ballPoint.x, ballPoint.y, ballPoint.z, bi.confidence);


	SpicaHelper::wm->ball = (bi);
	

	return ballImPoint;
}

void BallHelper::visualizeBall(unsigned char * src, int width, Point ball, int radius){
	//cout << "Endy Endy2CamPos x:" << ball.x << " y:" << ball.y << endl;
	int index = ((int)ball.y) + ((int)ball.x)*width;
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
#ifdef OLDBALL
	printf("OldBall: BlobBounds %d %d %d %d %d\n", oldBlob.left, oldBlob.right, oldBlob.top, oldBlob.bottom, oldBlob.count);
	if(oldBlob.left != 0){

		for(int i = oldBlob.left - 10; i <= oldBlob.right + 10; i++){
			src[(oldBlob.top - 10)*width + i] = 0;
			src[(oldBlob.bottom + 10)*width + i] = 0;
		}

		for(int i = oldBlob.top - 10; i <= oldBlob.bottom + 10; i++){
			src[i*width + (oldBlob.left - 10)] = 0;
			src[i*width + (oldBlob.right + 10)] = 0;
		}
	}
#endif
	// Visualize balls from directed camera
	ObservedPoint * opDirected = SharedMemoryHelper::getInstance()->readDirectedBallPosition();

	for(int i = 0; i < 10; i++){
		if(opDirected[i].valid){
			Point p = PositionHelper::getInstance()->getPoint3D2Cam(opDirected[i].x, opDirected[i].y, opDirected[i].z);			
	        	for(int i = p.y - 10; i <= p.y + 10; i++){
		                src[((int)p.x - 10)*width + i] = 255;
                		src[((int)p.x + 10)*width + i] = 255;
		        }

	        	for(int i = p.x - 10; i <= p.x; i++){
		                src[i*width + ((int)p.y - 10)] = 255;
                		src[i*width + ((int)p.y + 10)] = 255;
		        }
		
		}
	}

	// Visualize balls from Kinect
	opDirected = SharedMemoryHelper::getInstance()->readKinectBallPosition();
	for(int i = 0; i < 10; i++){
		if(opDirected[i].valid){
			Point p = PositionHelper::getInstance()->getPoint3D2Cam(opDirected[i].x, opDirected[i].y, opDirected[i].z);
			for(int i = p.y - 10; i <= p.y + 10; i++){
				src[((int)p.x - 10)*width + i] = 255;
				src[((int)p.x + 10)*width + i] = 255;
			}

			for(int i = p.x - 10; i <= p.x; i++){
				src[i*width + ((int)p.y - 10)] = 255;
				src[i*width + ((int)p.y + 10)] = 255;
			}
		}
	}
}


Point BallHelper::getBallPosition(){

	return currBallPos;

}

ObservedPoint BallHelper::getBallFromBlobsAdaptiveROI(ballCluster * cluster, int clusterCount, std::vector<ROIData>& roiData, std::vector<BlobBounds> & potBallBlobs, Particle * maxParticle){


	ObservedPoint op;

	bzero(&oldBlob, sizeof(BlobBounds));

	PositionHelper * posHelp = PositionHelper::getInstance();

	op.timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();
	op.x = 0.0;
	op.y = 0.0;
	op.z = 0.0;
	op.angleHori = 0.0;
	op.angleVerti = 0.0;
	op.confidence = 0.0;
	op.valid = false;

	std::vector<BlobBounds> ballBlobs;

	for(int i = 0; i < MAX_BALL_BLOBS; i++){

		ballProbs[i] = 1.0;

	}


	if(potBallBlobs.size() > 0){


		double minBallDistance = 30000.0;
		unsigned int ballIndex = potBallBlobs.size() + 1;
	
//		printf("////////////////////////////////////////////////\n");
	
		for(unsigned int i = 0; i < potBallBlobs.size(); i++){
#ifdef VERBOSE	
			printf("BallBlobWidth-%d: %d\n", i, potBallBlobs[i].right - potBallBlobs[i].left + 1);
			printf("BallBlobHeight-%d: %d\n", i, potBallBlobs[i].bottom - potBallBlobs[i].top + 1);
			printf("BallBlobMinDist-%d: %f\n", i, potBallBlobs[i].minDistance);
			printf("BallBlobCount-%d: %d\n", i, potBallBlobs[i].count);
#endif 

			double currBallAngle = -atan2((potBallBlobs[i].minY - MY),(potBallBlobs[i].minX - MX));
			double currBallPosX = cos(currBallAngle)*potBallBlobs[i].minDistance;
			double currBallPosY = sin(currBallAngle)*potBallBlobs[i].minDistance;
	
			bool inField = false;

			if(maxParticle == NULL || maxParticle->weight < LocalizationSuccess){
				inField = true;
			}
			else { 

	
				double alloBallPosX = maxParticle->posx;
				double alloBallPosY = maxParticle->posy;
	
				alloBallPosX += cos(maxParticle->heading)*currBallPosX - sin(maxParticle->heading)*currBallPosY;
				alloBallPosY += sin(maxParticle->heading)*currBallPosX + cos(maxParticle->heading)*currBallPosY;

				double relFactor = 250; //potBallBlobs[i].minDistance/4.0;
			/*	if(relFactor > 1500.0)
					relFactor = 1500.0;
				if(relFactor < 150.0)
					relFactor = 150.0;*/

				if(fabs(alloBallPosX) < FootballField::FieldLength/2.0 + relFactor && 
				 fabs(alloBallPosY) < FootballField::FieldWidth/2.0 + relFactor){
					inField = true;
				}

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

			ballProbs[i] *= sqrt(ratio);


			double coverage = potBallBlobs[i].count/(blobHeight*blobWidth);
			ballProbs[i] *= sqrt(coverage);			

			double expectedCount = potBallBlobs[i].minDistance*potBallBlobs[i].count/300000.0;
			if(expectedCount > 1.0)
				expectedCount = 1.0/expectedCount;
			ballProbs[i] *= sqrt(sqrt(expectedCount));

			if(potBallBlobs[i].minDistance < 300.0 && fabs(currBallAngle) > M_PI/6.0 && fabs(currBallAngle) < M_PI/2.0){
				printf("Ball could be in Kicker 2 or 3!\n");
				ballProbs[i] += 0.4;
			}

			if(potBallBlobs[i].minDistance < 300.0 && fabs(currBallAngle) > 5.0*M_PI/6.0){
				printf("Ball could be in Kicker 1!\n");
				ballProbs[i] += 0.4;

			}

			if(ballProbs[i] > 0.9)
				ballProbs[i] = 0.9;

#ifdef VERBOSE			
			printf("BallBlobProb-%d: %f\n", i, ballProbs[i]);
			printf("BallBlobProb-%d-ratio: %f\n", i, sqrt(ratio));
			printf("BallBlobProb-%d-coverage: %f\n", i, sqrt(coverage));
			printf("BallBlobProb-%d-expectedCount: %f\n\n", i, sqrt(sqrt(expectedCount)));
#endif
	
		}
	


		unsigned int maxProbIndex = potBallBlobs.size();

		minBallDistance = 30000.0;
		ballIndex = potBallBlobs.size();
		double maxProb = 0.0;
		double realBallProb = 0.0;
		for(unsigned int i = 0; i < potBallBlobs.size(); i++){
			if(ballProbs[i] > maxProb){
				maxProbIndex = i;
				maxProb = ballProbs[i];
			}
			if(ballProbs[i] > 0.35){
				if(potBallBlobs[i].minDistance < minBallDistance){
					minBallDistance = potBallBlobs[i].minDistance;
					ballIndex = i;
					realBallProb = ballProbs[i];
				}
			}

		}

		if(maxProbIndex < potBallBlobs.size()){

			int midX = (potBallBlobs[maxProbIndex].top + potBallBlobs[maxProbIndex].bottom)/2;
			int midY = (potBallBlobs[maxProbIndex].left + potBallBlobs[maxProbIndex].right)/2;

#ifdef VERBOSE		
			printf("+++++BallDistanceRoland: %f pixelCount: %d\n", potBallBlobs[maxProbIndex].minDistance, potBallBlobs[maxProbIndex].count);
			
			//double ballangle = -atan2((ballBlobs[0].minY - MY),(ballBlobs[0].minX - MX));
			double ballangle = -atan2((midY - MY), (midX - MX));
			printf("+++++BallAngleRoland: %f°\n\n", ballangle/M_PI*180.0);
#endif


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

			double ballLineAngle = atan2((midY - MY), (midX - MX));


			if(fabs(ballLineAngle) <= M_PI/4.0)
				ballBlobs[0].top = ballBlobs[0].bottom - diam + 1.0;			
			if(fabs(ballLineAngle) >= 3.0*M_PI/4.0)
				ballBlobs[0].bottom = ballBlobs[0].top + diam - 1.0;
			if(ballLineAngle > M_PI/4.0 && ballLineAngle < 3.0*M_PI/4.0)
				ballBlobs[0].left = ballBlobs[0].right - diam + 1.0;
			if(ballLineAngle < -M_PI/4.0 && ballLineAngle > -3.0*M_PI/4.0)
				ballBlobs[0].right = ballBlobs[0].left + diam - 1.0;

			midX = (ballBlobs[0].top + ballBlobs[0].bottom)/2;
			midY = (ballBlobs[0].left + ballBlobs[0].right)/2;


		
			printf("+++++BallDistance: %f pixelCount: %d\n", ballBlobs[0].minDistance, ballBlobs[0].count);
			
			//double ballangle = -atan2((ballBlobs[0].minY - MY),(ballBlobs[0].minX - MX));
			double ballangle = -atan2((midY - MY), (midX - MX));
			printf("+++++BallAngle: %f°\n\n", ballangle/M_PI*180.0);

			printf("+++++RealBallProb: %f\n", realBallProb);

			if(isGoalie && ballBlobs[0].minDistance*ballBlobs[0].count > 300000.0){
				printf("!!!!!!!!!!!!!!!!!!!!!!!!!!! BALL IS IN THE AIR !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
				ballBlobs[0].minDistance = 400000.0/ballBlobs[0].count;
			}


			Point3D p = posHelp->getPointCam2Point3D(midX, midY, diam/2.0);			

			oldBlob = ballBlobs[0];


			op.x = p.x; //cos(ballangle)*ballBlobs[0].minDistance;
			op.y = p.y; //sin(ballangle)*ballBlobs[0].minDistance;
			op.z = p.z;
			op.confidence = realBallProb;
			op.valid = true;


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








