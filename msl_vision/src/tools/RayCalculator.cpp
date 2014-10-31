/*
 * $Id: rayCalculator.cpp 1531 2013-06-10 16:19:57Z phbaer $
 *
 *
 * Copyright 2005-2013 Carpe Noctem, Distributed Systems Group,
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
 * Precalculate the ray for the 3D-Mirror-System
 *
 */

#include "RayCalculator.hpp"


// Initialize the camera
//std::string camera_model = "FL2G-13S2C"; // alternativ "FL2-08S2C"

// Initialize the objective.
//std::string objective_model;// = "DF6HA-1B";


struct ReflectionPoint
{
	double r;
	double z;
}reflectionPoint;

using std::cout;
using std::endl;
using std::flush;
using std::string;


int main ( int argc, char * argv[] )
{
	cout << "RayCalculator start" << endl;

	// Initialize the vision config files.
	supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
	string confPath = sc->getConfigPath();

	supplementary::Configuration *vision3D		= (*sc)["Vision3D"];
	if(vision3D==nullptr) return 0;

	cout << "Camera" << endl;
	cout << "\tModel_";
	string camera_model	= vision3D->get<std::string>("Camera", "Model", NULL);
	cout << "\tScanSize_";
	uint32_t scan_size	= vision3D->get<uint32_t>("Camera", camera_model.c_str(), "scan_size", NULL);
	cout << "\tPixelSize_";
	double pixel_size	= vision3D->get<double>("Camera", camera_model.c_str(), "pixel_size", NULL);	// [mm/px]

	cout << "Objective" << endl;
	cout << "\tModel_";
	string objective_model	= vision3D->get<std::string>("Objective", "Model", NULL);
	cout << "\tFocalLength_";
	double focal_length		= vision3D->get<double>("Objective", objective_model.c_str(), "focal_length", NULL);

// 	uint16_t max_resolution_width		= vision3D->get<uint16_t>("Camera", camera_model.c_str(), "max_resolution_width", NULL);
// 	uint16_t max_resolution_height	= vision3D->get<uint16_t>("Camera", camera_model.c_str(), "max_resolution_height", NULL);
	cout << "Image" << endl;
	cout << "\tWidth_";
	uint16_t width		= vision3D->get<uint16_t>("Image", "Width", NULL);
	cout << "\tHeight_";
	uint16_t height	= vision3D->get<uint16_t>("Image", "Height", NULL);

	cout << "Mirror" << endl;
	cout << "\tTransistionRadius_";
	uint16_t transition_radius	= vision3D->get<uint16_t>("Mirror", "TransitionRadius", NULL);
	cout << "\tHyperbelOffset_";
	double offsetH		= vision3D->get<double>("Mirror", "Hyperbel", "Offset", NULL);
	cout << "\tParabel1Offest_";
	double offsetP1	= vision3D->get<double>("Mirror", "Parabel_1", "Offset", NULL);
	cout << "\tParabel2Offset_";
	double offsetP2	= vision3D->get<double>("Mirror", "Parabel_2", "Offset", NULL);
	cout << "\tCorrectionAInner_";
	double correctionAInner = vision3D->get<double>("Mirror", "CorrectionAInner", NULL);
	cout << "\tCorrectionBInner_";
	double correctionBInner = vision3D->get<double>("Mirror", "CorrectionBInner", NULL);
	cout << "\tCorrectionAOuter_";
	double correctionAOuter = vision3D->get<double>("Mirror", "CorrectionAOuter", NULL);
	cout << "\tCorrectionBOuter_";
	double correctionBOuter = vision3D->get<double>("Mirror", "CorrectionBOuter", NULL);

	double focalpointH	= 120.0;
	double focalpointP1	= 0.0;
	double focalpointP2	= -8.84;

	double LookupTable[scan_size*2]; // Structure: a,b (Parameter for a slope z = a*r + b)

	// Calculate the max sensor radius in mm.
	double sensor_radius = sqrt(width*width + height*height) / 2.0;
	sensor_radius *= pixel_size;	// [mm = px * mm/px]

	double scan_step = sensor_radius / scan_size;	// [mm/increment]

	cout << "Building RayLookup.dat ... " << flush;

	// For the Radius 0
	LookupTable[0] = 0.0;
	LookupTable[1] = 0.0;

	for (uint32_t i=1; i<scan_size; i++)
	{
		// Calculate the radius in mm
		double radius = i * scan_step;	// [mm = increment * mm/increment]

		// Line function from sensor through focal point.
		// z = a*r + b
		// a := slope
		// b := focus distance ( = 0 )
		double slope = focal_length / radius; // a de/inclenation [mm/mm]

		// Calculating the reflection points.
		// Seperation for the two mirror views.
		if( radius < (transition_radius*pixel_size) )	// [mm < mm=pixel*mm/pixel]
		{
			// Reflection Point
			// Mirror function:
			// 25r² - 11z² + 1320z = 12100 [mm]
			double temp = 5*sqrt(11)*sqrt(1100*slope*slope + offsetH*offsetH + 120*offsetH + 1100);
			temp = 660*slope + 11*slope*offsetH + temp;
			reflectionPoint.r = temp / (11*slope*slope - 25);
			reflectionPoint.z = reflectionPoint.r * slope;

			// Calculate final slope.
			LookupTable[i*2]	= ((reflectionPoint.z - (focalpointH+offsetH)) / reflectionPoint.r) + correctionAInner;	// de/inclenation [(mm-mm)/mm]
			LookupTable[i*2+1]	= focalpointH+offsetH + correctionBInner;	// [mm]
		}
		else
		{
			// First Reflection Point.
			// Mirror function: z = -r²/2/2/130 + 130
			double temp = sqrt(67600*(slope*slope+1) + 520*offsetP1);
			reflectionPoint.r = temp - 260*slope;
			reflectionPoint.z = reflectionPoint.r*slope;

			// Calculate tangente angle.
			double angleT = atan(-reflectionPoint.r/260.0);
			// Calculate ray angle.
			double angleR = atan(slope);
			// New angle.
			double angle = 2.0*angleT-angleR;
			double vertical = -90*M_PI/180;

			if( (angle-vertical) > 1E-10 )
			{	// This code is not testet, it's a special case, if something crayz happens check this out.
				// Calculate the reflection slope
				slope = tan(angle);
				double b = reflectionPoint.z - slope*reflectionPoint.r;

				// Second Reflection
				// Mirror function: z = -r²/2/2/12.5 + 12.5 - 8.84
				double temp = sqrt(625*slope*slope - 50*b + 50.0*offsetP2 + 183.0);
				reflectionPoint.r = temp - 25.0*slope;
				reflectionPoint.z = reflectionPoint.r*slope + b;
			}
			else
			{
				// Second Reflection Point
				// Mirror function: z = -r²/2/2/12.5 + 12.5 - 8.84
				reflectionPoint.z = - reflectionPoint.r*reflectionPoint.r / 50.0 + 3.66 + offsetP2;
			}

			// Calculate final Slope.
			LookupTable[i*2]	= (reflectionPoint.z - (focalpointP2+offsetP2)) / reflectionPoint.r + correctionAOuter;
			LookupTable[i*2+1]	= focalpointP2+offsetP2 + correctionBOuter;
		}
	}

	cout << "." << std::flush;

	cout << endl << "RayLookup.dat was built" << endl;
	string filePath = confPath + "/RayLookup.dat";

	FILE *fd = fopen(filePath.c_str(), "w");

	// Preparing Checkdatas
	uint16_t tempU16 [3]	= {width, height, transition_radius};
	uint32_t tempU32 [1]	= {scan_size};
	float tempFloat [9]		= {(float)focal_length, (float)pixel_size, (float)offsetH, (float)offsetP1, (float)offsetP2, (float)correctionAInner, (float)correctionBInner, (float)correctionAOuter, (float)correctionBOuter};

	// Write Checkdatas
	fwrite(tempU16, sizeof(uint16_t), 3, fd);
	fwrite(tempU32, sizeof(uint32_t), 1, fd);
	fwrite(tempFloat, sizeof(float), 9, fd);

	fwrite(LookupTable, sizeof(double), scan_size*2, fd);

	fclose(fd);

	return 0;
}
