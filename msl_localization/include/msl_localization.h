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
#ifndef MSLLOCALIZATION_H
#define MSLLOCALIZATION_H

#include "RandomHelper.h"
#include "Types.h"
#include <vector>
#include "MapHelper.h"
#include <tf/transform_listener.h>
#include <msl_sensor_msgs/CorrectedOdometryInfo.h>
#include <msl_actuator_msgs/IMUData.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "Rprop.h"
#include "msl_sensor_msgs/LinePointList.h"
#include "SystemConfig.h"

#define RAWODOBUFSIZE 100


class msl_localization {

	public:
		msl_localization(int nParticles_);
		~msl_localization();

		void iterate(msl_sensor_msgs::LinePointListPtr & linePoints, unsigned char* distanceMap, msl_actuator_msgs::IMUDataPtr imu);
		void updateParticles(double deltaX, double deltaY, double deltaH); 
		Particle getMaxParticle();

		int getNumberParticles();
		Particle * getParticles();

		Particle getEstimatedPosition();
		
		double calculateWeightForEstimatedPosition(Position pos, msl_sensor_msgs::LinePointListPtr & linePoints, unsigned char *distanceMap);
		void writeCoi();
		
		void sendParticleCloud();
		void initParticles(double x, double y, double angle, double maxX, double maxY, double maxAngle);

		void resetStartParticle();

	protected:
		bool useOdometry;
		int minimizationSteps;
		bool minimize;
		RandomGaussHelper* gaussHelper;

		msl_actuator_msgs::RawOdometryInfo oldOdometryInfo;

		msl_sensor_msgs::CorrectedOdometryInfo coi;

		int nParticles;
		void initParticles();
		void resample(RandomGaussHelper & gaussHelper);
		void cleanup();
		void normalizeAngle(double &ang);

		Particle * particles;
		Particle maxParticle;
		Particle startParticle;

		Position rawUpdatedPosition;
		unsigned short msgid;
		
		bool isGoalie;

		double LocalizationSuccess;
		double LinePointSigma;
		bool UseRepParticles;
		bool UseBlueGoal;
		bool UseCornerPosts;
		
		double yellowGoalDirection;
		int xShift;
		int yShift;

		Position positionBuffer[RAWODOBUFSIZE];
		unsigned long long timestampBuffer[RAWODOBUFSIZE];

		int integrationIndex;
		bool bufferInitialized;
		bool reinit;

		MovingRobot mr;
		MovingRobot mrOld;

		int initCounter;
		tf::TransformBroadcaster tfBroadcaster;
		MapHelper *mh;
		int IHEIGHT, IWIDTH, IHEIGHT_2, IWIDTH_2; 
		double RESOLUTION;
};


#endif
