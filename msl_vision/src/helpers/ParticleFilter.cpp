/*
 * $Id: ParticleFilter.cpp 2865 2007-12-16 21:24:50Z cn $
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
#include "ParticleFilter.h"

#include "RandomHelper.h"
#include "FootballField.h"
#include "SpicaHelper.h"
#include "TimeHelper.h"
#include "BallIntegrator.h"
#include "EgoMotionEstimator.h"
//#include "PacketHelper.h"

#include <msl_sensor_msgs/CorrectedOdometryInfo.h>
#include <math.h>
#include <string.h>
#include <sys/time.h>

#define DISTNONE 20000.0
#define SIGMA 20

using namespace msl_sensor_msgs;
using namespace msl_msgs;

ParticleFilter::ParticleFilter(int nParticles_) : sc() {
	printf("ParticleFilter Constructor!\n");
	this->sc = SystemConfig::getInstance();
	FootballField::getInstance();

	yellowGoalDirection = -1;
	nParticles = nParticles_;
	lastIteration = 0;
	particles = NULL;

	rawOdometryHelper = RawOdometryHelper::getInstance();
	compassValueHelper = CompassValueHelper::getInstance();

	rawUpdatedPosition.x = 0.0;
	rawUpdatedPosition.y = 0.0;
	rawUpdatedPosition.heading = 0.0;

	for(int i = 0; i < RAWODOBUFSIZE; i++){
		positionBuffer[i] = rawUpdatedPosition;
		timestampBuffer[i] = 0;
	}

	integrationIndex = 0;
	bufferInitialized = false;
	msgid = 0;

	//isGoalie = (*this->sc)["Globals"]->get<bool>("Globals", "Team", SystemConfig::getHostname().c_str(), "IsGoalie", NULL);
	//printf("++++++++ System isGoalie %d\n", isGoalie);

	string currentField = (*this->sc)["FootballField"]->get<string>("FootballField", "CurrentField", NULL);
	yellowGoalDirection = (*this->sc)["FootballField"]->get<int>("FootballField", currentField.c_str(), "YellowGoalDirection", NULL);
	printf("++++++++ YellowGoalDirection %d\n", yellowGoalDirection);

	initParticles();

	LocalizationSuccess = (*this->sc)["Localization"]->get<double>("Localization", "LocalizationSuccess", NULL);
	LinePointSigma = (*this->sc)["Localization"]->get<double>("Localization", "LinePointSigma", NULL);
	UseRepParticles = (*this->sc)["Localization"]->get<bool>("Localization", "UseRepParticles", NULL);
	UseBlueGoal = (*this->sc)["Localization"]->get<bool>("Localization", "UseBlueGoal", NULL);
	UseCornerPosts = (*this->sc)["Localization"]->get<bool>("Localization", "UseCornerPosts", NULL);

	initCounter = 0;
    coi.certainty = (-1);
}


ParticleFilter::~ParticleFilter(){

	cleanup();


}


void inline ParticleFilter::normalizeAngle(double &ang)
{
	if(ang > M_PI)
		ang -= 2.0*M_PI;
	else if(ang < -M_PI)
		ang += 2.0*M_PI;
}


void ParticleFilter::iterate(std::vector<LinePoint> & linePoints, LineDistanceHelper & lineDistHelper, RandomGaussHelper & gaussHelper, std::vector<Goal> yellowGoals, std::vector<Goal> blueGoals, std::vector<CornerPost> cornerPosts, bool sendOdometry){

	struct timeval tv_before;
	struct timeval tv_after;
	long timediff;

	printf("Particlefilter MaxParticle Before Update: %f %f %f\n", maxParticle.posx, maxParticle.posy, maxParticle.heading);


	Position pos1 = rawOdometryHelper->getPositionData(lastIteration);
	Position pos2 = rawOdometryHelper->getPositionData();

	//Compute RawOdo Delta
	Position updatePos;
	updatePos.x = pos2.x - pos1.x;
	updatePos.y = pos2.y - pos1.y;
	updatePos.heading = pos2.heading - pos1.heading;
	normalizeAngle(updatePos.heading);


	updateParticles(updatePos.x, updatePos.y, updatePos.heading);

	printf("Particlefilter Particle Update Vector: %f %f %f\n", updatePos.x, updatePos.y, updatePos.heading);

	//Compute Max Weight/index before weightupdate
	int maxIndPre = 0;
	double maxWeightPre = particles[0].weight;
	for(int i = 1; i < nParticles; i++){
		if(particles[i].weight > maxWeightPre){
			maxIndPre = i;
			maxWeightPre = particles[i].weight;
		}
	}


	printf("MaxParticle After Update: %f %f %f\n", particles[maxIndPre].posx, particles[maxIndPre].posy, particles[maxIndPre].heading);



	unsigned char * linePointsInvalidity = (unsigned char *) malloc(linePoints.size());
	memset((void *) linePointsInvalidity, 0, linePoints.size());

	std::vector<LinePoint>::const_iterator first, last = linePoints.end();
	//int invCounter = 0;

	//Move particles according to rawUpdatedPosition
	double posConfidence = 0.0;
	for(int i = 0; i < nParticles; i++){

		double dx = particles[i].posx - rawUpdatedPosition.x;
		double dy = particles[i].posy - rawUpdatedPosition.y;
		if(sqrt(dx*dx + dy*dy) < 250.0 && particles[i].weight > posConfidence){
			posConfidence = particles[i].weight;
		}

	}


	//if position is at the field border
	/*if((fabs(rawUpdatedPosition.x) > FootballField::FieldLength/2.0 - 2000.0 || fabs(rawUpdatedPosition.y) > FootballField::FieldWidth/2.0 - 2000.0) && posConfidence > 0.4) {


		double cos_ = cos(rawUpdatedPosition.heading);
		double sin_ = sin(rawUpdatedPosition.heading);

		int invIndex = 0;

	//WTF???? isGoalie nach außen?
		for(first = linePoints.begin(); first != last; ++first){



			double realx = rawUpdatedPosition.x + cos_*(first->x) - sin_*(first->y);
			double realy = rawUpdatedPosition.y + sin_*(first->x) + cos_*(first->y);

			if(fabs(realx) > FootballField::FieldLength/2.0 + 800.0 || fabs(realy) > FootballField::FieldWidth/2.0 + 800.0){
				if(!isGoalie){
					linePointsInvalidity[invIndex] = 1;
					invCounter++;
				}

			}
			invIndex++;

		}

	}*/

	int compassValue = compassValueHelper->getCompassData();
	//printf("Particlefilter compass %d", compassValue);


	unsigned char * LineLookup = lineDistHelper.getLineLookup();

	//printf("LineDistance: %d\n", lineDistHelper.getLineDistance(-9000.0, 0.0));


	double t = 0.9999;
	//double h = (linePoints.size() - invCounter)*1.0;
	double h = (linePoints.size())*1.0;
	double pv = pow((t/(1-t)),(1/h))/(1 + pow((t/(1-t)),(1/h)));

	int IHEIGHT_2 = IHEIGHT/2;
	int IWIDTH_2 = IWIDTH/2;

	double resolution_1 = 1.0/RESOLUTION;

	int maxPosLinePoints = 0;
	int maxNegLinePoints = 0;
	double maxWeight = 0.0;

	gettimeofday(&tv_before, NULL);

	std::vector<unsigned char> linePointDistances;
	linePointDistances.clear();

	double * linePointsEgoDistances = (double *) malloc(linePoints.size()*sizeof(double));
	double * linePointsEgoAngles = (double *) malloc(linePoints.size()*sizeof(double));

	//Compute Ego LinePoint distance and Angle
	for(unsigned int i = 0; i < linePoints.size(); i++){
		double distance = sqrt(linePoints[i].x*linePoints[i].x + linePoints[i].y*linePoints[i].y);

		linePointsEgoDistances[i] = distance;
		linePointsEgoAngles[i] = atan2(linePoints[i].y, linePoints[i].x);

		//???
		double compare_distance = (LinePointSigma + (distance - 400.0)*0.005);
		int int_c_dist = lrint(compare_distance);
		if(int_c_dist < 0)
			int_c_dist = 0;
		if(int_c_dist > 255)
			int_c_dist = 255;

		unsigned char c = (unsigned char) int_c_dist;

		linePointDistances.push_back(c);


	}

	//Compute compassheading start
	double degrees = (((double) compassValue) / 10.0)*2.0*M_PI/360.0;

	double compX = cos(-degrees);
	double compY = sin(-degrees);

	double goalDegrees = (((double) yellowGoalDirection) / 10.0)*2.0*M_PI/360.0;
	double goalX = cos(-goalDegrees);
	double goalY = sin(-goalDegrees);

	// Is this the same as: compassHeading = goalDegrees - degrees; ?
	double compassHeading = atan2(compY, compX) - atan2(goalY, goalX);
	normalizeAngle(compassHeading);
	//Compute compassheading end


	for(int i = 0; i < nParticles; i++){

		//computes egoGoalPosts + distances
		/*Point dlP;
		dlP.x = -FootballField::FieldLength/2.0 - particles[i].posx;
		dlP.y = FootballField::GoalWidth/2.0 - particles[i].posy;

		double leftPostEgoDist = sqrt(dlP.x*dlP.x + dlP.y*dlP.y);
		double leftPostEgoAngle = atan2(dlP.y, dlP.x) - particles[i].heading;
		if(leftPostEgoAngle > M_PI)
			leftPostEgoAngle -= 2.0*M_PI;
		if(leftPostEgoAngle < -M_PI)
			leftPostEgoAngle += 2.0*M_PI;


		Point drP;
		drP.x = -FootballField::FieldLength/2.0 - particles[i].posx;
		drP.y = -FootballField::GoalWidth/2.0 - particles[i].posy;

		double rightPostEgoDist = sqrt(drP.x*drP.x + drP.y*drP.y);
		double rightPostEgoAngle = atan2(drP.y, drP.x) - particles[i].heading;
		if(rightPostEgoAngle > M_PI)
			rightPostEgoAngle -= 2.0*M_PI;
		if(rightPostEgoAngle < -M_PI)
			rightPostEgoAngle += 2.0*M_PI;
		*/


		//ego2allow of linePoints

		int posLinePoints = 0;
		int negLinePoints = 0;

		double cos_ = cos(particles[i].heading);
		double sin_ = sin(particles[i].heading);
		double realx = 0.0;
		double realy = 0.0;

		double weight = 0.5;
		unsigned char dist = 0;
		double inp = 0.0;


		//Do not Consider Particles with an angle difference to compass < pi/3
		if(yellowGoalDirection >= 0 && compassValue >= 0){
			double diffHeading = compassHeading - particles[i].heading;

			normalizeAngle(diffHeading);

			if(fabs(diffHeading) > M_PI/3.0){
				weight = 0.1;
				if(i == nParticles-2)
					printf("Loc1 : Drop UpdatedPosition\n");
				if(i == nParticles-1)
					printf("Loc1 : Drop MaxParticle %d %f %f, %f %f %f\n", compassValue, degrees, particles[i].heading, particles[i].posx, particles[i].posy, particles[i].weight);
			}

		}

		std::vector<LinePoint>::const_iterator first, last = linePoints.end();
		std::vector<unsigned char>::const_iterator firstDist, lastDist = linePointDistances.end();

		//If Particle has not been thrown away
		if(weight > 0.1) {

			int invIndex = 0;

			int tribotWeight = 0;

			//Iterate over Linepoints
			for(first = linePoints.begin(), firstDist = linePointDistances.begin(); first != last; ++first, ++firstDist){

				if(linePointsInvalidity[invIndex] == 0){

					//Transform LinePoint in coordinate system of current particle
					realx = particles[i].posx + cos_*(first->x) - sin_*(first->y);
					realy = particles[i].posy + sin_*(first->x) + cos_*(first->y);

					//Compute index in distance lookup
					int indX = lrint(-realy*resolution_1) + IHEIGHT_2;
					int indY = lrint(realx*resolution_1) + IWIDTH_2;

					//Compute "Distance value" [0-254]
					if(indX >= 0 && indX < IHEIGHT && indY >= 0 && indY < IWIDTH){
						dist = LineLookup[indX*IWIDTH + indY];
					}
					else dist = 255;

					//Count linepoints within a certain range
					/* Only for Old Weight
					if(dist < (*firstDist)){
						posLinePoints++;
					}*/

					/*else if(isGoalie){

						if(linePointsEgoDistances[invIndex] >= leftPostEgoDist && fabs(linePointsEgoAngles[invIndex] - leftPostEgoAngle) < M_PI/18.0)
							posLinePoints++;
						else if (linePointsEgoDistances[invIndex] >= rightPostEgoDist && fabs(linePointsEgoAngles[invIndex] - rightPostEgoAngle) < M_PI/18.0)
							posLinePoints++;


					}*/

					tribotWeight += dist;

					//weight = weight*inp/((1-weight)*(1-inp) + weight*inp);
				}

				invIndex++;
			}

			//negLinePoints = linePoints.size() - invCounter - posLinePoints;
			//Old Weight
			//negLinePoints = linePoints.size() - posLinePoints;

			//Recomputewight
			/* Old Weight
			inp = pv;
			if(posLinePoints > 0)
				weight = 1.0/(1.0 + pow(1-inp, posLinePoints)*(1-weight)/(weight*pow(inp, posLinePoints)));

			inp = 1.0 - pv;
			if(negLinePoints > 0)
				weight = 1.0/(1.0 + pow(1-inp, negLinePoints)*(1-weight)/(weight*pow(inp, negLinePoints)));
			*/

			//New "Tribotweight"
			//if((linePoints.size() - invCounter)>0)
			if((linePoints.size())>0)
			  //weight = 1.0 - tribotWeight/((linePoints.size() - invCounter)*510.0);
			  weight = 1.0 - tribotWeight/((linePoints.size())*510.0);
			else
			  weight = 0.1;

		}


		particles[i].weight = weight;
		if(weight > maxWeight) {
			maxWeight = weight;
			//Not Needed anymore?
			//maxNegLinePoints = negLinePoints;
			//maxPosLinePoints = posLinePoints;
		}
	}

	//gettimeofday(&tv_after, NULL);
//
	//timediff = tv_after.tv_usec - tv_before.tv_usec;
	//if(timediff < 0)
	//	timediff += 1000000;

	//printf("\n\nTime for Particles: %d\n\n", timediff);


	//Compute Best Particle + Index
	int maxInd = 1;
	maxWeight = 0;//particles[0].weight;
	for(int i = 1; i < nParticles; i++){
		if(particles[i].weight > maxWeight){
			maxInd = i;
			maxWeight = particles[i].weight;
		}
	}

	printf("ParticleFilter: posX: %f posY: %f heading %f prop: %f\n", particles[maxInd].posx, particles[maxInd].posy, particles[maxInd].heading, particles[maxInd].weight);
	memcpy(&maxParticle, &(particles[maxInd]), sizeof(Particle));

	//Update rawUpdatedPosition;
	Position rawPosition = rawOdometryHelper->getPositionData();

	double rotAngle = rawUpdatedPosition.heading - rawPosition.heading;
	normalizeAngle(rotAngle);

	double deltaX = cos(rotAngle)*updatePos.x - sin(rotAngle)*updatePos.y;
	double deltaY = sin(rotAngle)*updatePos.x + cos(rotAngle)*updatePos.y;

	rawUpdatedPosition.x += deltaX;
	rawUpdatedPosition.y += deltaY;
	rawUpdatedPosition.heading += updatePos.heading;
	normalizeAngle(rawUpdatedPosition.heading);

	bool jump = false;

	//After 150 Iterations, Successfull Lokalization for at least 75 linepoints
	if(initCounter >= 150 && maxParticle.weight > LocalizationSuccess && linePoints.size() > 75){
		bool updateAllowed = true;

		//If Position is within a corner
		if(fabs(rawUpdatedPosition.x) > FootballField::FieldLength/2.0 - FootballField::GoalAreaWidth &&
				fabs(rawUpdatedPosition.y) > FootballField::GoalAreaLength/2.0){

			double diffX = maxParticle.posx - rawUpdatedPosition.x;
			double diffY = maxParticle.posy - rawUpdatedPosition.y;

			//No Update if difference between rawUpdatedPosition and maxParticle > 300
			if(sqrt(diffX*diffX + diffY*diffY) > 300.0)
				updateAllowed = false;
		}

		double rawMaxWeight = 0.0;
		int maxTrackedIndex = nParticles - 1;

		for(int i = 0; i < nParticles; i++){
			double diffX = particles[i].posx - rawUpdatedPosition.x;
			double diffY = particles[i].posy - rawUpdatedPosition.y;

			//Compute Max particleweight within 50mm to rawUpdatesPosition
			if(sqrt(diffX*diffX + diffY*diffY) < 50.0){
				if(particles[i].weight > rawMaxWeight){
					rawMaxWeight = particles[i].weight;
					maxTrackedIndex = i;
				}
			}
		}

		//Update always allowed if weights < 0.4
		if(rawMaxWeight < 0.4)
			updateAllowed = true;

		double estimatedPositionWeight = particles[nParticles-2].weight;

		if(updateAllowed){
			//Compute distance and angle difference beteween rawUpdatedPosition and Maxparticle
			double headingDiff = fabs(maxParticle.heading - rawUpdatedPosition.heading);
			if(headingDiff > M_PI){
				headingDiff -= 2.0*M_PI;
			}

			if(fabs(headingDiff) > M_PI/360.0){
				rawUpdatedPosition.heading = maxParticle.heading;
			}

			double dist = sqrt((maxParticle.posx - rawUpdatedPosition.x)*(maxParticle.posx - rawUpdatedPosition.x) + (maxParticle.posy - rawUpdatedPosition.y)*(maxParticle.posy - rawUpdatedPosition.y));

			//Update rawUpdatePosition to best particle within 50mm
			rawUpdatedPosition.x = particles[maxTrackedIndex].posx;
			rawUpdatedPosition.y = particles[maxTrackedIndex].posy;

			//Believe more in the current position than positions > 5cm away
			double jumpThres = 0.02;
			if(estimatedPositionWeight > 0.98)
				jumpThres = 0.005;
			if(estimatedPositionWeight > 0.99)
				jumpThres = 0.001;

			//Update rawUpdatedPosition only farer away than 300mm if confidence is > estimatedPositionWeight + jumpthres
			if(dist > 300.0 && maxParticle.weight - estimatedPositionWeight >= jumpThres){
				rawUpdatedPosition.x = maxParticle.posx;
				rawUpdatedPosition.y = maxParticle.posy;
				rawUpdatedPosition.heading = maxParticle.heading;
				timestampBuffer[integrationIndex] = 0;
				jump = true;
			}
		}
	}

	//Write rawupdated Position + timestamp in ringbuffer
	if(bufferInitialized){
		integrationIndex++;
		if(integrationIndex >= RAWODOBUFSIZE)
			integrationIndex -= RAWODOBUFSIZE;

	}
	else {
		bufferInitialized = true;
	}

	positionBuffer[integrationIndex] = rawUpdatedPosition;
 	timestampBuffer[integrationIndex] = TimeHelper::getInstance()->getVisionTimeOmniCam();

	//Update Current Position and Velocity Estimation by Egomotionestimator
	EgoMotionEstimator * estimator = EgoMotionEstimator::getInstance();
	mrOld = mr;
	mr = estimator->trackObject(positionBuffer, timestampBuffer, RAWODOBUFSIZE, integrationIndex, 0.4E07);

	//If position or velocity jumped or if Velocity is very small -> Use old Velocity Estimation
	if(jump || (fabs(sqrt(mr.velocity.vx*mr.velocity.vx + mr.velocity.vy*mr.velocity.vy) - sqrt(mrOld.velocity.vx*mrOld.velocity.vx + mrOld.velocity.vy*mrOld.velocity.vy)) > 400.0 && sqrt(mr.velocity.vx*mr.velocity.vx + mr.velocity.vy*mr.velocity.vy) < 1.0E-32))
		mr.velocity = mrOld.velocity;

	int logTime = (int) ((TimeHelper::getInstance()->getVisionTimeOmniCam()/1000) % 1000000);
	if(logTime > 500000)
		logTime -= 1000000;

	printf("12345678911 %f %f %f %f %d\n", mr.velocity.vx, mr.velocity.vy, sqrt(mr.velocity.vx*mr.velocity.vx + mr.velocity.vy*mr.velocity.vy), mr.velocity.w, logTime);
	printf("12345678912 %f %f %f %d\n", mr.position.x, mr.position.y, mr.position.heading, logTime);

	//Send Odometry ^^
	if (sendOdometry) {
		Position rawUpdatedPositionNew = rawUpdatedPosition;

		PositionInfo robotPosition;
		MotionInfo robotVelocity;

		robotPosition.x = (mr.position.x);
		robotPosition.y = (mr.position.y);
		robotPosition.angle = (mr.position.heading);

		//Auskommentieren, wenn Delay zu groß!
		rawUpdatedPositionNew.x = mr.position.x;
		rawUpdatedPositionNew.y = mr.position.y;
		rawUpdatedPositionNew.heading = mr.position.heading;
		unsigned long long timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();

		int logTime = (int) ((timestamp/1000) % 1000000);
		if(logTime > 500000)
			logTime -= 1000000;

		printf("Particlefilter rawUpdatedPosition %f %f %f %d\n", rawUpdatedPositionNew.x, rawUpdatedPositionNew.y, rawUpdatedPositionNew.heading, logTime);
		printf("Particlefilter MaxParticle: %f %f %f\n", maxParticle.posx, maxParticle.posy, maxParticle.heading);

		robotVelocity.translation = (sqrt(mr.velocity.vx*mr.velocity.vx + mr.velocity.vy*mr.velocity.vy));
		robotVelocity.rotation = (mr.velocity.w);
		robotVelocity.angle = (atan2(mr.velocity.vy, mr.velocity.vx));

		coi.position = (robotPosition);
		coi.motion = (robotVelocity);
		//coi.certainty = (calculateWeightForEstimatedPosition(rawUpdatedPositionNew, linePoints, lineDistHelper, linePointsInvalidity, invCounter));
		coi.certainty = (calculateWeightForEstimatedPosition(rawUpdatedPositionNew, linePoints, lineDistHelper, linePointsInvalidity, 0));
		coi.locType.type = (LocalizationType::ParticleFilter);

		printf("ParticleFilter MaxParticle Confidence %f\n", coi.certainty);
#ifndef SWITCHLOC
		writeCoi();
#endif
		BallIntegrator::getInstance()->setRefPosition(rawUpdatedPositionNew);
	}

	if(particles[nParticles - 1].weight < 0.5)
		printf("Loc1: Previous MaxParticle with low confidence\n");
	if(particles[nParticles - 2].weight < 0.5)
		printf("Loc1: Previous RawUpdatedPos with low confidence\n");

	resample(gaussHelper);

	lastIteration = TimeHelper::getInstance()->getVisionTimeOmniCam();

	if(maxParticle.weight < LocalizationSuccess && linePoints.size() > 75){
		printf("Something going wrong with Localization!\n");
		printf("Cannot match Line Points!\n");
	}

	initCounter++;
	if(initCounter > 10000)
		initCounter = 10000;

	free(linePointsInvalidity);
	free(linePointsEgoDistances);
	free(linePointsEgoAngles);
}


void ParticleFilter::initParticles(){
	particles = (Particle *) malloc(nParticles*sizeof(Particle));

	RandomHelper::initRandom();

	double maxX = FootballField::FieldLength/2.0 + 500.0;
	double maxY = FootballField::FieldWidth/2.0 + 500.0;
	double maxAngle = M_PI;

	if(isGoalie){
		for(int i = 0; i < nParticles; i++){
			double rvalue = RandomHelper::rand01();
			if(rvalue > 0.5){
				particles[i].posx = -FootballField::FieldLength/2.0 - 500.0 + RandomHelper::rand01()*3500.0;
				particles[i].posy = (RandomHelper::rand01() - 0.5)*2.0*maxY;
				particles[i].heading = (RandomHelper::rand01() - 0.5)*2.0*maxAngle;
				particles[i].weight = 0.5;
			}
			else {
				particles[i].posx = +FootballField::FieldLength/2.0 + 500.0 - RandomHelper::rand01()*3500.0;
				particles[i].posy = (RandomHelper::rand01() - 0.5)*2.0*maxY;
				particles[i].heading = (RandomHelper::rand01() - 0.5)*2.0*maxAngle;
				particles[i].weight = 0.5;
			}
		}
	}
	else {
		for(int i = 0; i < nParticles; i++){
			particles[i].posx = (RandomHelper::rand01() - 0.5)*2.0*maxX;
			particles[i].posy = (RandomHelper::rand01() - 0.5)*2.0*maxY;
			particles[i].heading = (RandomHelper::rand01() - 0.5)*2.0*maxAngle;
			particles[i].weight = 0.5;
		}
	}
}

void ParticleFilter::resample(RandomGaussHelper & gaussHelper){
	double maxX = FootballField::FieldLength/2.0 + 500.0;
	double maxY = FootballField::FieldWidth/2.0 + 500.0;
	double maxAngle = M_PI;

	Particle repParticles[nParticles];
	unsigned int nRepParticles = 0;

	double distThreshold = -1.0;
	//if(UseRepParticles || isGoalie)
	if(UseRepParticles)
		distThreshold = 50.0*50.0;

	if(distThreshold < 0.0){
		//Count and store representative particles (weight > 0.9)
		for(int i = 0; i < nParticles; i++){
			if(particles[i].weight > 0.9){
				repParticles[nRepParticles] = particles[i];
				nRepParticles++;
			}
		}
	}
	else {
		for(int i = 0; i < nParticles; i++){
			if(particles[i].weight > 0.8){
				bool repFound = false;
				//?????????????????????????????? WTF
				for(unsigned int j = 0; j < nRepParticles; j++){
					double distance = (repParticles[j].posx - particles[i].posx)*(repParticles[j].posx - particles[i].posx) +
								(repParticles[j].posy - particles[i].posy)*(repParticles[j].posy - particles[i].posy);

					if(distance < distThreshold){
						repFound = true;
						if(particles[i].weight > repParticles[j].weight){
							repParticles[j] = particles[i];
							break;
						}
					}
				}
				if(!repFound){
					repParticles[nRepParticles] = particles[i];
					nRepParticles++;
				}
			}
		}
	}

	Particle * tmpParticles = (Particle *) malloc(nParticles*sizeof(Particle));
	memcpy(tmpParticles, particles, nParticles*sizeof(Particle));
	bool found = (nRepParticles > 0);
	double * cumWeights = (double *) malloc(nRepParticles*sizeof(double));
	double sum = 0.0;

	//Compute cumlative weight + normalization factor
	for(unsigned int i = 0; i < nRepParticles; i++){
		sum += repParticles[i].weight;
		cumWeights[i] = sum;
	}
	//Normalize Weights
	for(unsigned int i = 0; i < nRepParticles; i++)	cumWeights[i] /= sum;

	int nResample = 0;
	printf("Number RepParticles: %d\n", nRepParticles);

	//Mutate Max 400 Particles
	if(found){
		nResample = (nParticles*2)/3;
		if(nResample > 400)
			nResample = 400;
	}

	//Resample first 800
	for(int i = 0; i < nParticles - nResample; i++){
		particles[i].posx = (RandomHelper::rand01() - 0.5)*2.0*maxX;
		particles[i].posy = (RandomHelper::rand01() - 0.5)*2.0*maxY;
		particles[i].heading = (RandomHelper::rand01() - 0.5)*2.0*maxAngle;
		particles[i].weight = 0.5;
	}

	//Mutation of last 400
	for(int i = nParticles - nResample; i < nParticles; i++){
		double p = RandomHelper::rand01();
		unsigned int ind = 0;

		while(cumWeights[ind] < p && ind < nRepParticles)
			ind++;

		double sigma = (-2.0*repParticles[ind].weight + 2);
		if(sigma < 0.01)
			sigma = 0.01;
		particles[i].posx = repParticles[ind].posx + gaussHelper.getRandomGauss()*500.0*sigma;
		particles[i].posy = repParticles[ind].posy + gaussHelper.getRandomGauss()*500.0*sigma;
		particles[i].heading = repParticles[ind].heading + gaussHelper.getRandomGauss()*sigma*M_PI/4.0;
		particles[i].weight = 0.5;
	}

	//Last two Particles are MaxParticle and rawUpdatesPosition
	particles[nParticles-1].posx = maxParticle.posx;
	particles[nParticles-1].posy = maxParticle.posy;
	particles[nParticles-1].heading = maxParticle.heading;
	particles[nParticles-1].weight = maxParticle.weight;

	particles[nParticles-2].posx = rawUpdatedPosition.x;
	particles[nParticles-2].posy = rawUpdatedPosition.y;
	particles[nParticles-2].heading = rawUpdatedPosition.heading;
	particles[nParticles-2].weight = maxParticle.weight;

	free(tmpParticles);
	free(cumWeights);
}


void ParticleFilter::updateParticles(double deltaX, double deltaY, double deltaH){
	//Move all Particles by RawOdometry
	Position rawPosition = rawOdometryHelper->getPositionData(lastIteration);

	for(int i = 0; i < nParticles; i++){
		double rotAngle = particles[i].heading - rawPosition.heading;
		normalizeAngle(rotAngle);

		double newDeltaX = cos(rotAngle)*deltaX - sin(rotAngle)*deltaY;
		double newDeltaY = sin(rotAngle)*deltaX + cos(rotAngle)*deltaY;

		particles[i].posx += newDeltaX;
		particles[i].posy += newDeltaY;
		particles[i].heading += deltaH;
		normalizeAngle(particles[i].heading);
	}

	particles[0].posx = rawUpdatedPosition.x;
	particles[0].posy = rawUpdatedPosition.y;
	particles[0].heading = rawUpdatedPosition.heading;
	particles[0].weight = maxParticle.weight;

	particles[1] = maxParticle;
}


void ParticleFilter::cleanup(){
	if(particles != NULL){
		free(particles);
	}
}

Particle ParticleFilter::getMaxParticle(){
	return maxParticle;
}

int ParticleFilter::getNumberParticles(){
	return nParticles;
}

Particle * ParticleFilter::getParticles(){
	return particles;
}

WeightedPosition ParticleFilter::getEstimatedPosition(){
	WeightedPosition wPos;
	wPos.x = rawUpdatedPosition.x;
	wPos.y = rawUpdatedPosition.y;
	wPos.heading = rawUpdatedPosition.heading;
	wPos.weight = coi.certainty;
	return wPos;
}


double ParticleFilter::calculateWeightForEstimatedPosition(Position pos, std::vector<LinePoint> & linePoints, LineDistanceHelper & lineDistHelper, unsigned char * linePointsInvalidity, int invCounter){
	unsigned char * LineLookup = lineDistHelper.getLineLookup();

	SystemConfig* sysconf = SystemConfig::getInstance();
	double LinePointSigma = (*sysconf)["Localization"]->get<double>("Localization", "LinePointSigma", NULL);
	double offset = (*sysconf)["Localization"]->get<double>("Localization", "LinePointOffset", NULL);
	double t = 0.9999;
	double h = (linePoints.size() - invCounter)*1.0;
	double pv = pow((t/(1-t)),(1/h))/(1 + pow((t/(1-t)),(1/h)));

	int IHEIGHT_2 = IHEIGHT/2;
	int IWIDTH_2 = IWIDTH/2;

	double resolution_1 = 1.0/RESOLUTION;

	std::vector<LinePoint>::const_iterator first, last = linePoints.end();

	int posLinePoints = 0;
	int negLinePoints = 0;
	double cos_ = cos(pos.heading);
	double sin_ = sin(pos.heading);
	double realx = 0.0;
	double realy = 0.0;

	double weight = 0.5;
	unsigned char dist = 0;
	double inp = 0.0;

	if(weight > 0.1) {
		int invIndex = 0;

		int tribotWeight = 0;

		for(first = linePoints.begin(); first != last; ++first){
			double distance = sqrt((first->x)*(first->x) + (first->y)*(first->y));

			double compare_distance = (LinePointSigma + (distance - 400.0)*0.005);
			int int_c_dist = lrint(compare_distance);
			if(int_c_dist < 0)
				int_c_dist = 0;
			if(int_c_dist > 255)
				int_c_dist = 255;

			unsigned char c = (unsigned char) int_c_dist;

			if(linePointsInvalidity[invIndex] == 0){
				realx = pos.x + cos_*(first->x) - sin_*(first->y);
				realy = pos.y + sin_*(first->x) + cos_*(first->y);

					int indX = lrint(-realy*resolution_1) + IHEIGHT_2;
					int indY = lrint(realx*resolution_1) + IWIDTH_2;

					if(indX >= 0 && indX < IHEIGHT && indY >= 0 && indY < IWIDTH){
						dist = LineLookup[indX*IWIDTH + indY];
					}
					else
						dist = 255;

					if(dist >= 254)
						dist = 254;

					double dist_real = dist/255.0;
					double new_dist_real = sqrt((250.0*250.0)/(1.0 - dist_real) - 250.0*250.0)/10.0;

					new_dist_real = lrint(new_dist_real);
					if(new_dist_real >= 255.0)
						new_dist_real = 255.0;

					unsigned char new_dist = (unsigned char) lrint(new_dist_real);

					if(new_dist < c){
						posLinePoints++;
					}

					tribotWeight += dist;
					//weight = weight*inp/((1-weight)*(1-inp) + weight*inp);
				}
			invIndex++;
		}
		negLinePoints = linePoints.size() - invCounter - posLinePoints;

		inp = pv;
		if(posLinePoints > 0)
			weight = 1.0/(1.0 + pow(1-inp, posLinePoints)*(1-weight)/(weight*pow(inp, posLinePoints)));

		inp = 1.0 - pv;
		if(negLinePoints > 0)
			weight = 1.0/(1.0 + pow(1-inp, negLinePoints)*(1-weight)/(weight*pow(inp, negLinePoints)));

		//Endy The Hero
		//weight = 1.0 - tribotWeight/((linePoints.size() - invCounter)*510.0);
		if(invCounter==linePoints.size() || linePoints.size()==0) weight=0.1;
	}
	return weight;
}

void ParticleFilter::writeCoi()
{
	if (coi.certainty != -1 ) {
		unsigned long long timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();
		coi.imageTime = (timestamp);
		SpicaHelper::wm->odometry = (coi);
		printf("NewLoc PF im WM\n");
	}
	else printf("NewLoc: OOOPS no coi in particlefilter");
}
