/*
 * $Id: Calculator.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include <SystemConfig.h>

#include "distcalc.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <fstream>

//#define HEIGHT 480
//#define WIDTH 640
#define NSECTORS 360

using namespace supplementary;
using namespace std;

int main(int argc, char * argv[]){


	DistCalculator DC;

	SystemConfig* sc = SystemConfig::getInstance();
	std::string confPath = sc->getConfigPath();

	Configuration *vision = (*sc)["Vision"];
	if(vision==nullptr) {
		std::cout << "Error ... check source" << std::endl;
		return 0;
	}

	std::string filePath = confPath + sc->getHostname() + "/DistanceLookup.dat";
	FILE * fdtest = fopen(filePath.c_str(), "r");
	if( fdtest != NULL ) {
		std::cout << "Not Generating new File: File already exists: " << filePath << std::endl;
		return 0;
	}

	FILE * fd = fopen(filePath.c_str(), "w");
	if(fd == NULL) {
		std::cout << "Error creating " << filePath << std::endl;
		std::cout << "Cannot create File" << std::endl;
		return 0;
	}

	int MX;// = atoi((vision->Values["CameraMX"]).c_str());
	int MY;// = atoi((vision->Values["CameraMY"]).c_str());

	int CameraZ = vision->get<int>("Vision", "CameraZ", NULL);
	double g_m = vision->get<double>("Vision", "DistanceCalibM", NULL);
	double g_c = vision->get<double>("Vision", "DistanceCalibC", NULL);

	int SRadius = vision->get<double>("Vision", "CameraRadius", NULL);

	int dcHEIGHT = vision->get<int>("Vision", "ImageArea", NULL);
	int dcWIDTH = vision->get<int>("Vision", "ImageArea", NULL);

	DC.CD = vision->tryGet<int>(102.0, "Vision", "CD", NULL);

	MX = dcHEIGHT/2;
	MY = dcWIDTH/2;

	double LookupTable[dcHEIGHT][dcWIDTH];
	int LookupTableInt[dcHEIGHT][dcWIDTH][2];

	int X, Y;

	double D;

	std::cout << "CameraMX: " << MX << std::endl;
	std::cout << "CameraMY: " << MY << std::endl;
	std::cout << "DistanceCalibM: " << g_m << std::endl;
	std::cout << "DistanceCalibC: " << g_c << std::endl;
	std::cout << "CameraZ: " << CameraZ << std::endl;

	DC.setHeightOfCam(CameraZ);

	std::cout << "Building DistanceLookup.dat ..." << std::flush;

	for(int i = 0; i < dcHEIGHT; i++){
		for(int j = 0; j < dcWIDTH; j++){

			X = i - MX;
			Y = j - MY;

			D = sqrt((double)(X*X + Y*Y));
			if(D <= SRadius - 1){


				double ux = D*1.0/SRadius*R;
				double uy = DC.CD;

				double SX;
				double SY;

				double dist = DC.calcDistance(ux,uy, &SX, &SY);

				double newDist = dist*g_m + g_c;


				if(dist > 0.0){

					
					if(newDist > 0.0 && newDist < 150000.0){
						LookupTable[i][j] = newDist; //newDist;
						LookupTableInt[i][j][0] = (int) rint(newDist);
					}
					else{
						LookupTable[i][j] = newDist;
						LookupTableInt[i][j][0] = -1;				
					}	
				}
				else{
					LookupTable[i][j] = newDist;
					LookupTableInt[i][j][0] = -1;
				}
					

			}
			else{
				LookupTable[i][j] = 0.0;
				LookupTableInt[i][j][0] = -1;
			}

			LookupTableInt[i][j][1] = (int) floor((-atan2(Y,X) + M_PI)*NSECTORS/(2.0*M_PI));
			if(LookupTableInt[i][j][1] >= NSECTORS)
				LookupTableInt[i][j][1] = NSECTORS - 1;
		}
		std::cout << "." << std::flush;
		
	}


	fwrite(&(LookupTable[0][0]), sizeof(double), dcWIDTH*dcHEIGHT, fd);
	fwrite(&(LookupTableInt[0][0][0]), sizeof(int), dcWIDTH*dcHEIGHT*2, fd);
	fclose(fd);
	std::cout << std::endl << filePath << " was built" << std::endl;
	
	/*ofstream ofs("DistanceLookUp.txt");
	for(int i=0; i<dcWIDTH; i++) {
		for(int j=0; j<dcHEIGHT; j++) {
			ofs << LookupTable[i][j] << " ";
		}
		ofs << endl;
	}*/

	return 0;
}



