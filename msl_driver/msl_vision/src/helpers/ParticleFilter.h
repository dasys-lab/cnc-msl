/*
 * $Id: ParticleFilter.h 2863 2007-12-16 20:26:19Z rreichle $
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
#ifndef ParticleFilter_H
#define ParticleFilter_H

//#include <libAnja/DatagramSocket.h>
//#include <libAnja/UnixSocket.h>
#include "LinePoint.h"
#include "LineDistanceHelper.h"
#include "RandomHelper.h"
#include "RawOdometryHelper.h"
#include "CompassValueHelper.h"
#include "CornerPostHelper.h"
#include "../global/Types.h"

#include <vector>

#include <SystemConfig.h>
#include <CNSensorMsgs/CorrectedOdometryInfo.h>

using namespace castor;



class ParticleFilter {

	public:
		ParticleFilter(int nParticles_);
		~ParticleFilter();

		void iterate(std::vector<LinePoint> & linePoints, LineDistanceHelper & lineDistHelper, RandomGaussHelper & gaussHelper, std::vector<Goal> yellowGoals, std::vector<Goal> blueGoals, std::vector<CornerPost> cornerPosts, bool sendOdometry = true);
		void updateParticles(double deltaX, double deltaY, double deltaH); 
		Particle getMaxParticle();

		int getNumberParticles();
		Particle * getParticles();

		WeightedPosition getEstimatedPosition();
		
		static double calculateWeightForEstimatedPosition(Position pos, std::vector<LinePoint> & linePoints, LineDistanceHelper & lineDistHelper, unsigned char * linePointsInvalidity, int invCounter);
		void writeCoi();

	protected:

		SystemConfigPtr sc;

		CNSensorMsgs::CorrectedOdometryInfo coi;

		int nParticles;
		void initParticles();
		void resample(RandomGaussHelper & gaussHelper);
		void cleanup();
		void normalizeAngle(double &ang);

//		double calculateWeightForEstimatedPosition(Position pos, std::vector<LinePoint> & linePoints, LineDistanceHelper & lineDistHelper, unsigned char * linePointsInvalidity, int invCounter);

		Particle * particles;
		Particle maxParticle;

		RawOdometryHelper * rawOdometryHelper;
		CompassValueHelper * compassValueHelper;

//		Anja::Socket * socket;
//		std::string destAddress;
//		std::string socketType;
//		int destPort;

		Position rawUpdatedPosition;
		unsigned short msgid;
		
		bool isGoalie;

		double LocalizationSuccess;
		double LinePointSigma;
		bool UseRepParticles;
		bool UseBlueGoal;
		bool UseCornerPosts;
		
		int yellowGoalDirection;

		unsigned long long lastIteration;

		Position positionBuffer[RAWODOBUFSIZE];
		unsigned long long timestampBuffer[RAWODOBUFSIZE];

		int integrationIndex;
		bool bufferInitialized;

		MovingRobot mr;
		MovingRobot mrOld;

		

		int initCounter;


};


#endif
