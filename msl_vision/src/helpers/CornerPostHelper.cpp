/*
 * $Id: CornerPostHelper.cpp 1935 2007-03-19 19:50:12Z phbaer $
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
#include "CornerPostHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <sys/time.h>
//#include <GoalMessage.h>
//#include <CarpeNoctem/Messages/Information/YGoalInfo.h>
//#include <CarpeNoctem/Messages/Information/BGoalInfo.h>

//#include "../global/Packets.h"
//#include "PacketHelper.h"


CornerPostHelper::CornerPostHelper() : sc() {

	this->sc = SystemConfig::getInstance();

	MX = (*this->sc)["Vision.h"]->get<int>("Vision", "CameraMX", NULL);
	MY = (*this->sc)["Vision.h"]->get<int>("Vision", "CameraMY", NULL);

	init();


}


CornerPostHelper::~CornerPostHelper(){

	cleanup();

}



void CornerPostHelper::init(){


}


void CornerPostHelper::cleanup(){


}


void CornerPostHelper::findPosts(std::vector<BlobBounds> & yellowBlobs, std::vector<BlobBounds> & blueBlobs, std::vector<CornerPost> & cornerPosts){

	if(yellowBlobs.size() > 0 && blueBlobs.size() > 0){

		int * markedYellowBlobs = (int *) malloc(yellowBlobs.size()*sizeof(int));
		int * markedBlueBlobs = (int *) malloc(blueBlobs.size()*sizeof(int));
		bzero(markedYellowBlobs, yellowBlobs.size()*sizeof(int));
		bzero(markedBlueBlobs, blueBlobs.size()*sizeof(int));
	
		std::vector<BlobBounds> newYellowBlobs;
		newYellowBlobs.clear();
	
		std::vector<BlobBounds> newBlueBlobs;
		newBlueBlobs.clear();
	
		std::vector<double> postAngles;
		postAngles.clear();
	
	
		for(unsigned int i = 0; i < yellowBlobs.size(); i++){
	
			double yellowAngle = atan2(yellowBlobs[i].minY - MY, yellowBlobs[i].minX - MX);
	
	
			for(unsigned int j = 0; j < blueBlobs.size(); j++){
	
				double blueAngle = atan2(blueBlobs[j].minY - MY, blueBlobs[j].minX - MX);
	
				double weightRatio = ((double) yellowBlobs[i].count)/((double) blueBlobs[j].count);
	
				if(weightRatio >= 0.25 && weightRatio <= 4.0){
	
					double diffAngle = yellowAngle - blueAngle;
					if(diffAngle > M_PI)
						diffAngle -= 2.0*M_PI;
					if(diffAngle < -M_PI)
						diffAngle += 2.0*M_PI;
	
					if(fabs(diffAngle) < M_PI/10.0){
						markedYellowBlobs[i] = 1;
						markedBlueBlobs[j] = 1;
	
						double postAngle = yellowAngle + diffAngle/2.0;
						if(postAngle > M_PI)
							postAngle -= 2.0*M_PI;
						if(postAngle < -M_PI)
							postAngle += 2.0*M_PI;
						postAngles.push_back(postAngle);
	
					}
	
				}
		
			}
		}

		std::vector<double> newPostAngles;
		newPostAngles.clear();

		printf("PostAngles.size() = %d before\n", (int)postAngles.size());
		
		for(unsigned int i = 0; i < postAngles.size(); i++){

			bool found = false;
			for(unsigned int j = 0; j< newPostAngles.size(); j++){
				double diffAngle = newPostAngles[j] - postAngles[i];
				if(diffAngle < -M_PI)
					diffAngle += 2.0*M_PI;
				if(diffAngle > M_PI)
					diffAngle -= 2.0*M_PI;

				if(fabs(diffAngle) < M_PI/20.0)
					found = true;

			}
			if(!found)
				newPostAngles.push_back(postAngles[i]);
		}

		postAngles.swap(newPostAngles);

		int blueCounter[postAngles.size()];
		bzero(blueCounter, sizeof(int)*postAngles.size());
		int yellowCounter[postAngles.size()];
		bzero(yellowCounter, sizeof(int)*postAngles.size());
		double minDistances[postAngles.size()];
		int postWeights[postAngles.size()];
		bzero(postWeights, sizeof(int)*postAngles.size());
//		bzero(minDistances, sizeof(double)*postAngles.size());
		for(unsigned int i = 0; i < postAngles.size(); i++){
			minDistances[i] = 20000.0;

		}

		for(unsigned int i = 0; i < yellowBlobs.size(); i++){

			double angle = atan2(yellowBlobs[i].minY - MY, yellowBlobs[i].minX - MX);
			for(unsigned int j = 0; j < postAngles.size(); j++){
				double diffAngle = angle - postAngles[j];
				if(diffAngle < -M_PI)
					diffAngle += 2.0*M_PI;
				if(diffAngle > M_PI)
					diffAngle -= 2.0*M_PI;
				if(diffAngle < M_PI/18.0){
					yellowCounter[j]++;
					postWeights[j] += yellowBlobs[i].count;
					if(minDistances[j] > yellowBlobs[i].minDistance)
						minDistances[j] = yellowBlobs[i].minDistance;
				}
			}

		}

		for(unsigned int i = 0; i < blueBlobs.size(); i++){

			double angle = atan2(blueBlobs[i].minY - MY, blueBlobs[i].minX - MX);
			for(unsigned int j = 0; j < postAngles.size(); j++){
				double diffAngle = angle - postAngles[j];
				if(diffAngle < -M_PI)
					diffAngle += 2.0*M_PI;
				if(diffAngle > M_PI)
					diffAngle -= 2.0*M_PI;
				if(diffAngle < M_PI/18.0){
					blueCounter[j]++;
					postWeights[j] += blueBlobs[i].count;
					if(minDistances[j] > blueBlobs[i].minDistance)
						minDistances[j] = blueBlobs[i].minDistance;

				}

			}

		}
		
		cornerPosts.clear();	

		printf("PostAngles.size() = %d\n", (int)postAngles.size());

		int maxWeight = 0;
		for(unsigned int i = 0; i < postAngles.size(); i++){
			if(postWeights[i] > maxWeight)
				maxWeight = postWeights[i];
		}
		

		for(unsigned int i = 0; i < postAngles.size(); i++){
			
			if(yellowCounter[i] > blueCounter[i] && postWeights[i] > (int) rint(maxWeight*2.0/3.0)){
				printf("Found Yellow CornerPost at %f Distance %f Weight %d\n", postAngles[i], minDistances[i], postWeights[i]);
				CornerPost post;
				post.angle = postAngles[i];
				post.distance = minDistances[i];
				post.yellow = true;
				cornerPosts.push_back(post);
			}
			if(yellowCounter[i] < blueCounter[i] && postWeights[i] > (int) rint(maxWeight*2.0/3.0)){
				printf("Found Blue CornerPost at %f Distance %f Weight %d\n", postAngles[i], minDistances[i], postWeights[i]);
				CornerPost post;
				post.angle = postAngles[i];
				post.distance = minDistances[i];
				post.yellow = false;
				cornerPosts.push_back(post);
			}
			

		}
	
		for(unsigned int i = 0; i < yellowBlobs.size(); i++){
			if(markedYellowBlobs[i] == 0){
				newYellowBlobs.push_back(yellowBlobs[i]);
			}
		}
	
		yellowBlobs.swap(newYellowBlobs);
	
		for(unsigned int i = 0; i < blueBlobs.size(); i++){
			if(markedBlueBlobs[i] == 0){
				newBlueBlobs.push_back(blueBlobs[i]);
			}
		}
	
		blueBlobs.swap(newBlueBlobs);
	
	
		free(markedYellowBlobs);
		free(markedBlueBlobs);
	
	}

}

