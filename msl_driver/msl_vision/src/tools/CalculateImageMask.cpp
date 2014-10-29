/*
 * $Id: ReduceImageArea.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include <fstream>
#include <iostream>
#include <string>

#include <SystemConfig.h>

#define RADIUS 220
#define INNERRADIUS 60
#define HOLDER 3.5

using namespace castor;

int main(int argc, char * argv[]){

	//Calculate Image Mask

	SystemConfigPtr sc = SystemConfig::getInstance();

	Configuration *vision = (*sc)["Vision"];

	int area = vision->get<int>("Vision", "ImageArea", NULL);

	int mx = vision->get<int>("Vision", "CameraMX", NULL);
	int my = vision->get<int>("Vision", "CameraMY", NULL);

	mx = area/2;
	my = area/2;


	unsigned char LookupTable[area][area];

	memset((void *) LookupTable, 255, area*area);

	unsigned char * CenterMask = NULL;
	unsigned char dx = 0;
	unsigned char dy = 0;


	std::string file_name = sc->getRootPath() + "/src/Vision5/tools/CenterMask.dat";

	FILE * fd = fopen(file_name.c_str(), "r");
	if(fd != NULL){

		int n1 = fread(&dx, sizeof(unsigned char), 1, fd);
		int n2 = fread(&dy, sizeof(unsigned char), 1, fd);

		printf("dx = %d read %d\n", dx, n1);
		printf("dy = %d read %d\n", dy, n2);

		CenterMask = (unsigned char*) malloc((2*dx+1)*(2*dy+1));

		fread(CenterMask, sizeof(unsigned char), (2*dx+1)*(2*dy+1), fd);

		fclose(fd);
	}
	else{
		printf("CenterMask file not found ....\n");
		printf("file name: %s\n", file_name.c_str());
		exit(1);
	}

	printf("Calculating Image Mask ...................................\n");


	//out of circle

	for(int i = 0; i < area; i++){
		for(int j = 0; j < area; j++){

			int x = i - mx;
			int y = j - my;

			double d = sqrt((double)(x*x + y*y));
			if(d > RADIUS){

				LookupTable[i][j] = 0;
			}
			if(d < INNERRADIUS){
				
				LookupTable[i][j] = 128;

			}
		}
		std::cout << "." << std::flush;
		
	}

	//CenterMask 

	for(int i = -dx; i <= dx; i++){
		for(int j = -dy; j <= dy; j++){

			int xm = i + dx;
			int ym = j + dy;

			int x = mx + i;
			int y = my + j;

			//LookupTable[x][y] = 0;

			if(CenterMask[xm*(2*dy+1) + ym] < 1){
				LookupTable[x][y] = 0; //CenterMask[xm*(2*dy+1) + ym];
			}
			//if(CenterMask[xm*dy + ym] < 1){
			//	LookupTable[x][y] = 0;
			//}
		}
	}




	//holders

	for(int i = 0; i < area; i++){
		for(int j = 0; j < area; j++){

			int x = i - mx;
			int y = j - my;

			double angle = fabs(atan2(y, x))*180.0/M_PI;
			double angleDiff = fabs(angle - 120.0);

			if( angle <= HOLDER || angleDiff <= HOLDER){

				LookupTable[i][j] = 0;
			}
		}
		std::cout << "." << std::flush;
		
	}


	std::cout << std::endl << "Image Mask calculated" << std::endl;

	fd = fopen("ImageMask.dat", "w");
	fwrite(&(LookupTable[0][0]), sizeof(char), area*area, fd);
	fclose(fd);
	

	return 0;
}



