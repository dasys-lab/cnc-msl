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
#include "ParticleFilterGoalie.h"

#include "RandomHelper.h"
#include "FootballField.h"
#include "SpicaHelper.h"
#include "TimeHelper.h"
#include "BallIntegrator.h"
#include "EgoMotionEstimator.h"
#include "SharedMemoryHelper.h"
//#include "PacketHelper.h"

#include <msl_sensor_msgs/CorrectedOdometryInfo.h>
#include <math.h>
#include <string.h>
#include <sys/time.h>

#define DISTNONE 20000.0
#define SIGMA 20

using namespace msl_sensor_msgs;
using namespace msl_msgs;

ParticleFilterGoalie::ParticleFilterGoalie(int nParticles_) : sc() {

	this->sc = SystemConfig::getInstance();

	printf("ParticleFilter Constructor!\n");

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

	isGoalie = false; //(*this->sc)["Globals"]->get<bool>("Globals", "Team", asio::ip::host_name().c_str(), "IsGoalie", NULL);

	isGoalie2 = (*this->sc)["Globals"]->get<bool>("Globals", "Team", SystemConfig::getHostname().c_str(), "IsGoalie", NULL);

	printf("++++++++ System isGoalie %d\n", isGoalie);
	printf("++++++++ System isGoalie2 %d\n", isGoalie2);

	yellowGoalDirection = (*this->sc)["Globals"]->get<int>("Globals", "FootballField", "YellowGoalDirection", NULL);

	printf("++++++++ YellowGoalDirection %d\n", yellowGoalDirection);

	initParticles();

	printf("ParticleFilter Constructor!\n");

	LocalizationSuccess = (*this->sc)["Localization"]->get<double>("Localization", "LocalizationSuccess", NULL);
	LinePointSigma = (*this->sc)["Localization"]->get<double>("Localization", "LinePointSigma", NULL);
	UseRepParticles = (*this->sc)["Localization"]->get<bool>("Localization", "UseRepParticles", NULL);
	UseBlueGoal = (*this->sc)["Localization"]->get<bool>("Localization", "UseBlueGoal", NULL);
	UseCornerPosts = (*this->sc)["Localization"]->get<bool>("Localization", "UseCornerPosts", NULL);

	initCounter = 0;

	coi.certainty = (-1);

}

ParticleFilterGoalie::~ParticleFilterGoalie(){

	cleanup();


}


void ParticleFilterGoalie::iterate(std::vector<LinePoint> & linePoints, LineDistanceHelper & lineDistHelper, RandomGaussHelper & gaussHelper, std::vector<Goal> yellowGoals, std::vector<Goal> blueGoals, std::vector<CornerPost> cornerPosts, bool sendOdometry){


	struct timeval tv_before;
	struct timeval tv_after;
	long timediff;

	printf("MaxParticle Before Update: %f %f %f\n", maxParticle.posx, maxParticle.posy, maxParticle.heading);


	Position pos1 = rawOdometryHelper->getPositionData(lastIteration);
	Position pos2 = rawOdometryHelper->getPositionData();

	Position updatePos;
	updatePos.x = pos2.x - pos1.x;
	updatePos.y = pos2.y - pos1.y;
	updatePos.heading = pos2.heading - pos1.heading;
	if(updatePos.heading > M_PI)
		updatePos.heading -= 2.0*M_PI;
	if(updatePos.heading < -M_PI)
		updatePos.heading += 2.0*M_PI;



//rawOdometryHelper->getUpdateVectorAndReset();


	updateParticles(updatePos.x, updatePos.y, updatePos.heading);

	printf("Particle Update Vector: %f %f %f\n", updatePos.x, updatePos.y, updatePos.heading);


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

	int invCounter = 0;

	double posConfidence = 0.0;
	for(int i = 0; i < nParticles; i++){

		double dx = particles[i].posx - rawUpdatedPosition.x;
		double dy = particles[i].posy - rawUpdatedPosition.y;
		if(sqrt(dx*dx + dy*dy) < 250.0 && particles[i].weight > posConfidence){
			posConfidence = particles[i].weight;
		}

	}



	if((fabs(rawUpdatedPosition.x) > FootballField::FieldLength/2.0 - 1000.0 || fabs(rawUpdatedPosition.y) > FootballField::FieldWidth/2.0 - 1000.0) && posConfidence > 0.4) {


		double cos_ = cos(rawUpdatedPosition.heading);
		double sin_ = sin(rawUpdatedPosition.heading);

		int invIndex = 0;


		for(first = linePoints.begin(); first != last; ++first){



			double realx = rawUpdatedPosition.x + cos_*(first->x) - sin_*(first->y);
			double realy = rawUpdatedPosition.y + sin_*(first->x) + cos_*(first->y);

			if(fabs(realx) > FootballField::FieldLength/2.0 + 500.0 || fabs(realy) > FootballField::FieldWidth/2.0 + 500.0){
				if(!isGoalie){
					linePointsInvalidity[invIndex] = 1;
					invCounter++;
				}

			}
			invIndex++;

		}

	}

	int compassValue = -1;
	if(sendOdometry)
		compassValue = compassValueHelper->getCompassData();
	else
		compassValue = compassValueHelper->getCompassData2();


	unsigned char * LineLookup = lineDistHelper.getLineLookup();

	//printf("LineDistance: %d\n", lineDistHelper.getLineDistance(-9000.0, 0.0));


	double t = 0.9999;
	double h = (linePoints.size() - invCounter)*1.0;
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

	for(unsigned int i = 0; i < linePoints.size(); i++){
		double distance = sqrt(linePoints[i].x*linePoints[i].x + linePoints[i].y*linePoints[i].y);

		linePointsEgoDistances[i] = distance;
		linePointsEgoAngles[i] = atan2(linePoints[i].y, linePoints[i].x);

		double compare_distance = (LinePointSigma + (distance - 400.0)*0.005);
		int int_c_dist = lrint(compare_distance);
		if(int_c_dist < 0)
			int_c_dist = 0;
		if(int_c_dist > 255)
			int_c_dist = 255;

		unsigned char c = (unsigned char) int_c_dist;

		linePointDistances.push_back(c);


	}



	for(int i = 0; i < nParticles; i++){

		//egoGoalPosts
		Point dlP;
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


		if(yellowGoalDirection >= 0 && compassValue >= 0){

			double degrees = (((double) compassValue) / 10.0)*2.0*M_PI/360.0;

			double compX = cos(-degrees);
			double compY = sin(-degrees);

			double goalDegrees = (((double) yellowGoalDirection) / 10.0)*2.0*M_PI/360.0;
			double goalX = cos(-goalDegrees);
			double goalY = sin(-goalDegrees);

			double compassHeading = atan2(compY, compX) - atan2(goalY, goalX);
			if(compassHeading > M_PI)
				compassHeading -= 2.0*M_PI;
			if(compassHeading < -M_PI)
				compassHeading += 2.0*M_PI;



			double diffHeading = compassHeading - particles[i].heading;

			if(diffHeading > M_PI)
				diffHeading -= 2.0*M_PI;
			if(diffHeading < -M_PI)
				diffHeading += 2.0*M_PI;

			if(fabs(diffHeading) > M_PI/2.0)
				weight = 0.1;



		}


		std::vector<LinePoint>::const_iterator first, last = linePoints.end();
		std::vector<unsigned char>::const_iterator firstDist, lastDist = linePointDistances.end();

		if(weight > 0.1) {

			int invIndex = 0;

			int tribotWeight = 0;

			for(first = linePoints.begin(), firstDist = linePointDistances.begin(); first != last; ++first, ++firstDist){

				if(linePointsInvalidity[invIndex] == 0){

					realx = particles[i].posx + cos_*(first->x) - sin_*(first->y);
					realy = particles[i].posy + sin_*(first->x) + cos_*(first->y);

					int indX = lrint(-realy*resolution_1) + IHEIGHT_2;
					int indY = lrint(realx*resolution_1) + IWIDTH_2;

					if(indX >= 0 && indX < IHEIGHT && indY >= 0 && indY < IWIDTH){
						dist = LineLookup[indX*IWIDTH + indY];
						//printf("Distance: %d %f %f %d %d\n", dist, realx, realy, indX, indY);
					}
					else
						//dist = MAX_LDIST;
						dist = 255;
					//inp = pv;
					if(dist < (*firstDist)){
					//if(dist > SIGMA){
						posLinePoints++;
						//inp = 1.0 - pv;
					}

					else if(false && isGoalie){

						if(linePointsEgoDistances[invIndex] >= leftPostEgoDist && fabs(linePointsEgoAngles[invIndex] - leftPostEgoAngle) < M_PI/18.0)
							posLinePoints++;
						else if (linePointsEgoDistances[invIndex] >= rightPostEgoDist && fabs(linePointsEgoAngles[invIndex] - rightPostEgoAngle) < M_PI/18.0)
							posLinePoints++;


					}

					tribotWeight += dist;
	/*				else{
						posLinePoints++;
					}*/

					//weight = weight*inp/((1-weight)*(1-inp) + weight*inp);

				}


				invIndex++;
			}

			negLinePoints = linePoints.size() - invCounter - posLinePoints;

			//printf("PosLinePoints : %d NegLinePoints : %d\n", posLinePoints, negLinePoints);

			inp = pv;
			if(posLinePoints > 0)
				weight = 1.0/(1.0 + pow(1-inp, posLinePoints)*(1-weight)/(weight*pow(inp, posLinePoints)));

			inp = 1.0 - pv;
			if(negLinePoints > 0)
				weight = 1.0/(1.0 + pow(1-inp, negLinePoints)*(1-weight)/(weight*pow(inp, negLinePoints)));

			weight = 1.0 - tribotWeight/((linePoints.size() - invCounter)*510.0);

		}


		particles[i].weight = weight;
		if(weight > maxWeight){
			maxWeight = weight;
			maxNegLinePoints = negLinePoints;
			maxPosLinePoints = posLinePoints;
		}

	}

	gettimeofday(&tv_after, NULL);

	timediff = tv_after.tv_usec - tv_before.tv_usec;
	if(timediff < 0)
		timediff += 1000000;

	//printf("\n\nTime for Particles: %d\n\n", timediff);


	int maxInd = 0;
	maxWeight = particles[0].weight;
	for(int i = 1; i < nParticles; i++){
		if(particles[i].weight > maxWeight){
			maxInd = i;
			maxWeight = particles[i].weight;
		}
	}

	std::vector<Particle> repParticles;
	repParticles.clear();

	for(int i = 0; i < nParticles; i++){
		if(particles[i].weight > maxWeight - 0.02){

			bool repFound = false;
			for(unsigned int j = 0; j < repParticles.size(); j++){

				double distance = (repParticles[j].posx - particles[i].posx)*(repParticles[j].posx - particles[i].posx) +
							(repParticles[j].posy - particles[i].posy)*(repParticles[j].posy - particles[i].posy);

				if(distance < 200.0*200.0){
					repFound = true;
					if(particles[i].weight > repParticles[j].weight){
						repParticles[j] = particles[i];
						break;
					}
				}
			}

			if(!repFound)
				repParticles.push_back(particles[i]);

		}
	}

/*	for(unsigned int i = 0; i < repParticles.size(); i++){

		printf("RepParticle posx=%f posy=%f heading=%f weight=%f\n", repParticles[i].posx, repParticles[i].posy, repParticles[i].heading, repParticles[i].weight);


	}*/
	if(sendOdometry)
		printf("ParticleFilter: posX: %f posY: %f heading %f  %d prop: %f \n", particles[maxInd].posx, particles[maxInd].posy, particles[maxInd].heading, linePoints.size(), particles[maxInd].weight);
	//printf("ParticleFilter: posLinePoints: %d negLinePoints: %d\n", maxPosLinePoints, maxNegLinePoints);

	memcpy(&maxParticle, &(particles[maxInd]), sizeof(Particle));


	Position rawPosition = rawOdometryHelper->getPositionData();
	//printf("RawPosition: %f %f %f\n", rawPosition.x, rawPosition.y, rawPosition.heading);

	double rotAngle = rawUpdatedPosition.heading - rawPosition.heading;
	if(rotAngle > M_PI){
		rotAngle -= 2.0*M_PI;
	}
	if(rotAngle < -M_PI){
		rotAngle += 2.0*M_PI;
	}

	double deltaX = cos(rotAngle)*updatePos.x - sin(rotAngle)*updatePos.y;
	double deltaY = sin(rotAngle)*updatePos.x + cos(rotAngle)*updatePos.y;

	rawUpdatedPosition.x += deltaX;
	rawUpdatedPosition.y += deltaY;
	rawUpdatedPosition.heading += updatePos.heading;

	if(rawUpdatedPosition.heading > M_PI){
		rawUpdatedPosition.heading -= 2.0*M_PI;
	}

	if(rawUpdatedPosition.heading < -M_PI){
		rawUpdatedPosition.heading += 2.0*M_PI;
	}


	bool jump = false;

	double estimatedPositionWeight = 0.0;

	if(initCounter > 300 && maxParticle.weight > LocalizationSuccess && linePoints.size() > 75){


		bool updateAllowed = true;

		if(fabs(rawUpdatedPosition.x) > FootballField::FieldLength/2.0 - FootballField::GoalAreaWidth && fabs(rawUpdatedPosition.y) > FootballField::GoalAreaLength/2.0){

			double diffX = maxParticle.posx - rawUpdatedPosition.x;
			double diffY = maxParticle.posy - rawUpdatedPosition.y;

			if(sqrt(diffX*diffX + diffY*diffY) > 300.0)
				updateAllowed = false;


		}

		double rawMaxWeight = 0.0;
		int maxTrackedIndex = nParticles - 1;

		for(int i = 0; i < nParticles; i++){

			double diffX = particles[i].posx - rawUpdatedPosition.x;
			double diffY = particles[i].posy - rawUpdatedPosition.y;

			if(sqrt(diffX*diffX + diffY*diffY) < 50.0){

				if(particles[i].weight > rawMaxWeight){
					rawMaxWeight = particles[i].weight;
					maxTrackedIndex = i;
				}
			}


		}

		if(rawMaxWeight < 0.4)
			updateAllowed = true;

		estimatedPositionWeight = particles[nParticles-2].weight;


		if(updateAllowed){

/*			if(fabs(maxParticle.posx - rawUpdatedPosition.x) > 150.0){
				rawUpdatedPosition.x = maxParticle.posx;
			}

			if(fabs(maxParticle.posy - rawUpdatedPosition.y) > 150.0){
				rawUpdatedPosition.y = maxParticle.posy;
			}
*/
			double headingDiff = fabs(maxParticle.heading - rawUpdatedPosition.heading);
			if(headingDiff > M_PI){
				headingDiff -= 2.0*M_PI;

			}

			if(fabs(headingDiff) > M_PI/360.0){
				rawUpdatedPosition.heading = maxParticle.heading;
			}

			double dist = sqrt((maxParticle.posx - rawUpdatedPosition.x)*(maxParticle.posx - rawUpdatedPosition.x) + (maxParticle.posy - rawUpdatedPosition.y)*(maxParticle.posy - rawUpdatedPosition.y));


			rawUpdatedPosition.x = particles[maxTrackedIndex].posx;
			rawUpdatedPosition.y = particles[maxTrackedIndex].posy;

			double distThres = 300.0;
			if(isGoalie2)
				distThres = 150.0;

			if(dist > distThres && (isGoalie2 || (maxParticle.weight - estimatedPositionWeight >= 0.02))){
				rawUpdatedPosition.x = maxParticle.posx;
				rawUpdatedPosition.y = maxParticle.posy;
				rawUpdatedPosition.heading = maxParticle.heading;
				timestampBuffer[integrationIndex] = 0;
				jump = true;
			}

		}

	}

	if(sendOdometry){
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

		printf("RawUpdatedPosition prop: %f %f %f %f\n", rawUpdatedPosition.x, rawUpdatedPosition.y, rawUpdatedPosition.heading, estimatedPositionWeight);

		EgoMotionEstimator * estimator = EgoMotionEstimator::getInstance();

		mrOld = mr;

		mr = estimator->trackObject(positionBuffer, timestampBuffer, RAWODOBUFSIZE, integrationIndex, 0.4E07);

		if(jump || (fabs(sqrt(mr.velocity.vx*mr.velocity.vx + mr.velocity.vy*mr.velocity.vy) - sqrt(mrOld.velocity.vx*mrOld.velocity.vx + mrOld.velocity.vy*mrOld.velocity.vy)) > 400.0 && sqrt(mr.velocity.vx*mr.velocity.vx + mr.velocity.vy*mr.velocity.vy) < 1.0E-32))
			mr.velocity = mrOld.velocity;

		int logTime = (int) ((TimeHelper::getInstance()->getVisionTimeOmniCam()/1000) % 1000000);
		if(logTime > 500000)
			logTime -= 1000000;


		printf("12345678911 %f %f %f %f %d\n", mr.velocity.vx, mr.velocity.vy, sqrt(mr.velocity.vx*mr.velocity.vx + mr.velocity.vy*mr.velocity.vy), mr.velocity.w, logTime);

		printf("12345678912 %f %f %f %d\n", mr.position.x, mr.position.y, mr.position.heading, logTime);
	}


	if (sendOdometry) {

		Position rawUpdatedPositionNew = rawUpdatedPosition;
// rawOdometryHelper->updatePositionWithOdoData(rawUpdatedPosition);

		PositionInfo robotPosition;//
		MotionInfo robotVelocity;//

		robotPosition.x = (mr.position.x);
		robotPosition.y = (mr.position.y);
		robotPosition.angle = (mr.position.heading);

		printf("prop sent position: %f %f %f\n", mr.position.x, mr.position.y, mr.position.heading);


/*		robotPosition->setX(0.0);
		robotPosition->setY(0.0);
		robotPosition->setAngle(0.0);
*/
		unsigned long long timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();

		int logTime = (int) ((timestamp/1000) % 1000000);
		if(logTime > 500000)
			logTime -= 1000000;

		printf("1234567893 %f %f %f %d\n", rawUpdatedPositionNew.x, rawUpdatedPositionNew.y, rawUpdatedPositionNew.heading, logTime);


		//printf("Particle Filter: Sent Position: %f %f %f\n", rawUpdatedPosition.x, rawUpdatedPosition.y, rawUpdatedPosition.heading);
		printf("MaxParticle Send: %f %f %f\n", maxParticle.posx, maxParticle.posy, maxParticle.heading);

		robotVelocity.translation = (sqrt(mr.velocity.vx*mr.velocity.vx + mr.velocity.vy*mr.velocity.vy));
		robotVelocity.rotation = (mr.velocity.w);
		robotVelocity.angle = (atan2(mr.velocity.vy, mr.velocity.vx));

		//coi = CorrectedOdometryInfo::create();

		coi.position = (robotPosition);
		coi.motion = (robotVelocity);
		coi.certainty = (calculateWeightForEstimatedPosition(rawUpdatedPositionNew, linePoints, lineDistHelper, linePointsInvalidity, invCounter));

#ifndef SWITCHLOC
		writeCoi();
#endif
		//SpicaHelper::wm->getData()->push_back(coi);

		BallIntegrator::getInstance()->setRefPosition(rawUpdatedPositionNew);



		CorrectedOdometry corrOdo;
		corrOdo.posX = mr.position.x;
		corrOdo.posY = mr.position.y;
		corrOdo.posAngle = mr.position.heading;
		corrOdo.posCertainty = maxParticle.weight;

		SharedMemoryHelper::getInstance()->writeCorrectedOdometry(&corrOdo);

/*
 * Spica bug
 * if (SpicaHelper::visionCEP) {
std::cout << "Spica: create COI" << std::endl;
			CorrectedOdometryMessagePtr com = CorrectedOdometryMessage::create();
			com->setCoi(coi);
			try {
			SpicaHelper::visionCEP->getCom().insert(com, ServicePtr());
			} catch (const std::exception &e) {
				std::cout << "Spica " << e.what() << std::endl;
			}
std::cout << "Spica: done COI" << std::endl;
		}
*/
	}



	gettimeofday(&tv_before, NULL);


	resample(gaussHelper);

	gettimeofday(&tv_after, NULL);

	timediff = tv_after.tv_usec - tv_before.tv_usec;
	if(timediff < 0)
		timediff += 1000000;

	//printf("\n\nTime for resampling: %d\n\n", timediff);

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


void ParticleFilterGoalie::initParticles(){

	particles = (Particle *) malloc(nParticles*sizeof(Particle));

	RandomHelper::initRandom();

	double maxX = FootballField::FieldLength/2.0 + 500.0;
	double maxY = FootballField::FieldWidth/2.0 + 500.0;
	double maxAngle = M_PI;


	if(isGoalie2 && false){
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


void ParticleFilterGoalie::resample(RandomGaussHelper & gaussHelper){

	double maxX = FootballField::FieldLength/2.0 + 500.0;
	double maxY = FootballField::FieldWidth/2.0 + 500.0;
	double maxAngle = M_PI;

	Particle repParticles[nParticles];
	unsigned int nRepParticles = 0;

	double distThreshold = -1.0;
	if(UseRepParticles || isGoalie)
		distThreshold = 50.0*50.0;


	struct timeval tv_before;
	gettimeofday(&tv_before, NULL);


	if(distThreshold < 0.0){

		for(int i = 0; i < nParticles; i++){
			if(particles[i].weight > 0.8){

				repParticles[nRepParticles] = particles[i];
				nRepParticles++;

			}
		}

	}
	else {


		for(int i = 0; i < nParticles; i++){
			if(particles[i].weight > 0.8){

				bool repFound = false;
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



	struct timeval tv_after;
	gettimeofday(&tv_after, NULL);

	long timediff = tv_after.tv_usec - tv_before.tv_usec;
	if(timediff < 0)
		timediff += 1000000;

	//printf("\n\nTime for RepParticles: %d\n\n", timediff);


	Particle * tmpParticles = (Particle *) malloc(nParticles*sizeof(Particle));

	memcpy(tmpParticles, particles, nParticles*sizeof(Particle));

	bool found = (nRepParticles > 0);

	double * cumWeights = (double *) malloc(nRepParticles*sizeof(double));

	double sum = 0.0;


	for(unsigned int i = 0; i < nRepParticles; i++){

		sum += repParticles[i].weight;
		cumWeights[i] = sum;

	}

	for(unsigned int i = 0; i < nRepParticles; i++){
		cumWeights[i] /= sum;
	}

	int nResample = 0;

	if(found){

		nResample = (nParticles*2)/3;
		if(nResample > 100)
			nResample = 100;

	}


	if(false && isGoalie2){

		//double goalieOffset = - FootballField::FieldLength/2.0 - 500.0;

		for(int i = 0; i < nParticles - nResample; i++){
			double rvalue = RandomHelper::rand01();
			if(rvalue > 0.5){
				particles[i].posx = -FootballField::FieldLength/2.0 - 500.0 + RandomHelper::rand01()*3500.0;
				particles[i].posy = (RandomHelper::rand01() - 0.5)*2.0*maxY;
				particles[i].heading = rawUpdatedPosition.heading + gaussHelper.getRandomGauss()*M_PI/18.0;
				particles[i].weight = 0.5;
			}
			else {
				particles[i].posx = +FootballField::FieldLength/2.0 + 500.0 - RandomHelper::rand01()*3500.0;
				particles[i].posy = (RandomHelper::rand01() - 0.5)*2.0*maxY;
				particles[i].heading = rawUpdatedPosition.heading + gaussHelper.getRandomGauss()*M_PI/18.0;
				particles[i].weight = 0.5;
			}
		}

	}
	else {

		for(int i = 0; i < nParticles - nResample; i++){
			particles[i].posx = (RandomHelper::rand01() - 0.5)*2.0*maxX;
			particles[i].posy = (RandomHelper::rand01() - 0.5)*2.0*maxY;
			particles[i].heading = (RandomHelper::rand01() - 0.5)*2.0*maxAngle;
			particles[i].weight = 0.5;
		}

	}


	for(int i = nParticles - nResample; i < nParticles; i++){

		double p = RandomHelper::rand01();
		unsigned int ind = 0;

		while(cumWeights[ind] < p && ind < nRepParticles)
			ind++;

		double sigma = 0.0; //(-2.0*repParticles[ind].weight + 2);

		if(isGoalie2)
			sigma = 2.0*(-2.0*repParticles[ind].weight + 2);
		else
			sigma = (-2.0*repParticles[ind].weight + 2);

		if(sigma < 0.01)
			sigma = 0.01;

		particles[i].posx = repParticles[ind].posx + gaussHelper.getRandomGauss()*500.0*sigma;
		particles[i].posy = repParticles[ind].posy + gaussHelper.getRandomGauss()*500.0*sigma;
		particles[i].heading = rawUpdatedPosition.heading + gaussHelper.getRandomGauss()*sigma*M_PI/4.0;
		particles[i].weight = 0.5;


	}

	particles[nParticles-1].posx = maxParticle.posx;
	particles[nParticles-1].posy = maxParticle.posy;
	particles[nParticles-1].heading = maxParticle.heading;
	particles[nParticles-1].weight = maxParticle.weight;

	particles[nParticles-2].posx = rawUpdatedPosition.x;
	particles[nParticles-2].posy = rawUpdatedPosition.y;
	particles[nParticles-2].heading = rawUpdatedPosition.heading;
	particles[nParticles-2].weight = maxParticle.weight;


	/*particles[nParticles-1].posx = -8300.0; //maxParticle.posx;
	particles[nParticles-1].posy = 400.0; //maxParticle.posy;
	particles[nParticles-1].heading = 1.85; //maxParticle.heading;
	particles[nParticles-1].weight = 0.99; //maxParticle.weight;*/


	free(tmpParticles);
	free(cumWeights);

}





void ParticleFilterGoalie::updateParticles(double deltaX, double deltaY, double deltaH){

	Position rawPosition = rawOdometryHelper->getPositionData(lastIteration);

//	printf("UP: rawPos head %f\n", rawPosition.heading);
//	printf("UP: deltaX: %f deltaY: %f\n", deltaX, deltaY);

	for(int i = 0; i < nParticles; i++){

//		printf("UP: part head %f\n", particles[i].heading);

		double rotAngle = particles[i].heading - rawPosition.heading;
		if(rotAngle > M_PI){
			rotAngle -= 2.0*M_PI;
		}

		if(rotAngle < -M_PI){
			rotAngle += 2.0*M_PI;

		}

		double newDeltaX = cos(rotAngle)*deltaX - sin(rotAngle)*deltaY;
		double newDeltaY = sin(rotAngle)*deltaX + cos(rotAngle)*deltaY;

//		printf("UP: newV deltaX: %f deltaY: %f\n", newDeltaX, newDeltaY);

		particles[i].posx += newDeltaX;
		particles[i].posy += newDeltaY;
		particles[i].heading += deltaH;
		if(particles[i].heading > M_PI)
			particles[i].heading -= 2.0*M_PI;
		if(particles[i].heading < -M_PI)
			particles[i].heading += 2.0*M_PI;
	}


	if(isGoalie2) {
		particles[0] = maxParticle;
		particles[1].posx = 0.0;
		particles[1].posy = FootballField::FieldWidth/2.0;
		particles[1].heading = 0.0;
		particles[2].posx = 0.0;
		particles[2].posy = -FootballField::FieldWidth/2.0;
		particles[2].heading = 0.0;

	}

}


void ParticleFilterGoalie::cleanup(){

	if(particles != NULL){
		free(particles);
	}
}

Particle ParticleFilterGoalie::getMaxParticle(){


	return maxParticle;
}

int ParticleFilterGoalie::getNumberParticles(){

	return nParticles;

}

Particle * ParticleFilterGoalie::getParticles(){

	return particles;

}

WeightedPosition ParticleFilterGoalie::getEstimatedPosition(){
        WeightedPosition wPos;
        wPos.x = rawUpdatedPosition.x;
        wPos.y = rawUpdatedPosition.y;
        wPos.heading = rawUpdatedPosition.heading;
        wPos.weight = maxParticle.weight;
        return wPos;
}



double ParticleFilterGoalie::calculateWeightForEstimatedPosition(Position pos, std::vector<LinePoint> & linePoints, LineDistanceHelper & lineDistHelper, unsigned char * linePointsInvalidity, int invCounter){

	unsigned char * LineLookup = lineDistHelper.getLineLookup();

	double t = 0.9999;
	double h = (linePoints.size() - invCounter)*1.0;
	double pv = pow((t/(1-t)),(1/h))/(1 + pow((t/(1-t)),(1/h)));

	SystemConfig* sysconf = SystemConfig::getInstance();
	double LinePointSigma = (*sysconf)["Localization"]->get<double>("Localization", "LinePointSigma", NULL);
	double offset = (*sysconf)["Localization"]->get<double>("Localization", "LinePointOffset", NULL);

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

				// check if linepoints within fieldboundings
//				if (realx - offset > -FootballField::FieldLength/2 || realx + offset < FootballField::FieldLength/2 || realy - offset > -FootballField::FieldWidth/2 || realy+offset < FootballField::FieldWidth/2) {

					int indX = lrint(-realy*resolution_1) + IHEIGHT_2;
					int indY = lrint(realx*resolution_1) + IWIDTH_2;

					if(indX >= 0 && indX < IHEIGHT && indY >= 0 && indY < IWIDTH){
						dist = LineLookup[indX*IWIDTH + indY];
						//printf("Distance: %d %f %f %d %d\n", dist, realx, realy, indX, indY);
					}
					else
						//dist = MAX_LDIST;
						dist = 255;
					//inp = pv;

					if(dist >= 254)
						dist = 254;

					double dist_real = dist/255.0;
					double new_dist_real = sqrt((250.0*250.0)/(1.0 - dist_real) - 250.0*250.0)/10.0;

					new_dist_real = lrint(new_dist_real);
					if(new_dist_real >= 255.0)
						new_dist_real = 255.0;

					unsigned char new_dist = (unsigned char) lrint(new_dist_real);



					if(new_dist < c){
					//if(dist > SIGMA){
						posLinePoints++;
						//inp = 1.0 - pv;
					}

	/*				else{
						posLinePoints++;
					}*/

					//weight = weight*inp/((1-weight)*(1-inp) + weight*inp);
				}
//			}


			invIndex++;
		}

		negLinePoints = linePoints.size() - invCounter - posLinePoints;

		//printf("PosLinePoints : %d NegLinePoints : %d\n", posLinePoints, negLinePoints);

		inp = pv;
		if(posLinePoints > 0)
			weight = 1.0/(1.0 + pow(1-inp, posLinePoints)*(1-weight)/(weight*pow(inp, posLinePoints)));

		inp = 1.0 - pv;
		if(negLinePoints > 0)
			weight = 1.0/(1.0 + pow(1-inp, negLinePoints)*(1-weight)/(weight*pow(inp, negLinePoints)));


	}

	return weight;


}

void ParticleFilterGoalie::writeCoi()
{
	if (coi.certainty != -1 ) {
		unsigned long long timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();
		coi.imageTime = (timestamp);
		SpicaHelper::wm->odometry = (coi);
		printf("NewLoc PF im WM\n");
	}
	else printf("SwitchLoc: OOOPS no coi in particlefilter");
}


