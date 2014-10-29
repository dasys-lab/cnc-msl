
#include "Distance3DHelper.hpp"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <valarray>
#include <SystemConfig.h>

using std::cout;
using std::endl;
using std::string;

extern bool simulation;

Distance3DHelper::Distance3DHelper()
{
	cout << "Start Distance3DHelper Constructor" << endl;
	
	/// Read Lookup3D
	// Initialize the vision config files.
	castor::SystemConfigPtr sc = castor::SystemConfig::getInstance();
	string confPath = sc->getConfigPath();
	
	castor::Configuration *vision3D	= (*sc)["Vision3D"];
	
	cout << "Image" << endl;
	cout << "\tWidth_";
	width	= vision3D->get<uint16_t>("Image", "Width", NULL);
	cout << "\tHeight_";
	height	= vision3D->get<uint16_t>("Image", "Height", NULL);
	cout << "\tPannoWidth_";
	pannoWidth	= vision3D->get<uint16_t>("Image", "Panno", "Width", NULL);
	
	cout << "Camera" << endl;
	cout << "\tModel_";
	string 	camera_model	= vision3D->get<std::string>("Camera", "Model", NULL);
	cout << "\tScanSize_";
	uint32_t 	scan_size		= vision3D->get<uint32_t>("Camera", camera_model.c_str(), "scan_size", NULL);
	cout << "\tPixelSize_";
	
	cout << "Localization" << endl;
	cout << "\tCamera?Height";
	cameraHeight = vision3D->get<float>("Localization", "CameraHeight", NULL);

	pixel_size	= vision3D->get<float>("Camera", camera_model.c_str(), "pixel_size", NULL);	// [mm/pixel]
	
	cout << "Objective" << endl;
	cout << "\tModel_";
	string objective_model	= vision3D->get<string>("Objective", "Model", NULL);
	cout << "\tFocalLength_";
	float focal_length	= vision3D->get<float>("Objective", objective_model.c_str(), "focal_length", NULL);	
	
	cout << "ScanLines" << endl;
	cout << "\tInnerRadiusStart_";
	iRadiusStart	= vision3D->get<uint16_t>("ScanLines", "InnerRadiusStart", NULL);
	cout << "\tInnerRadiusEnd_";
	iRadiusEnd	= vision3D->get<uint16_t>("ScanLines", "InnerRadiusEnd", NULL);
	cout << "\tOuterRadiusStart_";
	oRadiusStart	= vision3D->get<uint16_t>("ScanLines", "OuterRadiusStart", NULL);
	cout << "\tOuterRadiusEnd_";
	oRadiusEnd	= vision3D->get<uint16_t>("ScanLines", "OuterRadiusEnd", NULL);
	
	cout << "Mirror" << endl;
	cout << "\tTransitionRadius_";
	uint16_t transition_radius	= vision3D->get<uint16_t>("Mirror", "TransitionRadius", NULL);
	cout << "\tHyperbelOffset_";
	float offsetH		= vision3D->get<float>("Mirror", "Hyperbel", "Offset", NULL);
	cout << "\tParabel1Offset_";
	float offsetP1		= vision3D->get<float>("Mirror", "Parabel_1", "Offset", NULL);
	cout << "\tParabel2Offest_";
	float offsetP2		= vision3D->get<float>("Mirror", "Parabel_2", "Offset", NULL);
	cout << "\tCorrectionAInner_";
	float correctionAInner	= vision3D->get<float>("Mirror", "CorrectionAInner", NULL);
	cout << "\tCorrectionBInner_";
	float correctionBInner	= vision3D->get<float>("Mirror", "CorrectionBInner", NULL);
	cout << "\tCorrecitonAOuter_";
	float correctionAOuter	= vision3D->get<float>("Mirror", "CorrectionAOuter", NULL);
	cout << "\tCorrectionBOuter_";
	float correctionBOuter	= vision3D->get<float>("Mirror", "CorrectionBOuter", NULL);
	
	/// Get scan step	
	sensor_radius	= sqrt(width*width + height*height) / 2;	// pixel
	sensor_radius *= pixel_size;		// [mm=pixel*mm/pixel]
	scan_step	= sensor_radius / scan_size;	// [mm/increment]
	
	// Preparing Checkdatas
	uint16_t tempU16 [3];
	uint32_t tempU32 [1];
	float tempFloat[9];

	string filePath = confPath + "/RayLookup.dat";
	FILE *fd = fopen(filePath.c_str(), "r");
	
	// Read Checkdata
	fread(tempU16, sizeof(uint16_t), 3, fd);
	fread(tempU32, sizeof(uint32_t), 1, fd);
	fread(tempFloat, sizeof(float), 9, fd);
	
	// Check Data
	if( (tempU16[0]!=width) || (tempU16[1]!=height) || (tempU16[2]!=transition_radius) || (tempU32[0]!=scan_size) ||
		(tempFloat[0]!=focal_length) || (tempFloat[1]!=(float)pixel_size) || (tempFloat[2]!=offsetH) || (tempFloat[3]!=offsetP1) ||
		(tempFloat[4]!=offsetP2) || (tempFloat[5]!=correctionAInner) || (tempFloat[6]!=correctionBInner) || (tempFloat[7]!=correctionAOuter) || (tempFloat[8]!=correctionBOuter))
	{
		cout << tempFloat[0] << " " << focal_length << endl;
		cout << tempFloat[1] << " " << pixel_size << endl;
		cout << tempFloat[2] << " " << offsetH << endl;
		cout << tempFloat[3] << " " << offsetP1 << endl;
		cout << tempFloat[4] << " " << offsetP2 << endl;
		cout << tempFloat[5] << " " << correctionAInner << endl;
		cout << tempFloat[6] << " " << correctionBInner << endl;
		cout << tempFloat[7] << " " << correctionAOuter << endl;
		cout << tempFloat[8] << " " << correctionBOuter << endl;
		
		printf("\n\nRebuild necessary, parameter changed.\n\n");
		exit(EXIT_FAILURE);
	}
	
	Lookup3D		= new double [scan_size*2];
	
	if( fread(Lookup3D, sizeof(double), scan_size*2, fd) == NULL)
	printf("Can't open %s\n", filePath.c_str());
	fclose(fd);
	
	cout << "End Distance3DHelper Constructor" << endl;
}



Point Distance3DHelper::getDistancePanno(uint16_t const yPannoInner, uint16_t const yPannoOuter) const
{
	uint32_t	index;
	uint16_t	x;
	uint16_t	y;
	double	radius;
	double	slopeA1;
	double	slopeB1;
	double	slopeA2;
	double	slopeB2;
	
	/// Get inner slope [z=a*r+b]
	// Calculate radius
	radius	= iRadiusEnd - yPannoInner;			// [pixel]
	index	= round(radius*pixel_size/scan_step)*2;	// [increment = pixel * mm/pixel / (mm/increment)]
	slopeA1	= Lookup3D[index];		// deInclination
	slopeB1	= Lookup3D[index+1];	// zero-offset [mm]
	
	/// Get outer slope [z=a*r+b]
	// Calculate radius
	radius	= yPannoOuter - (iRadiusEnd - iRadiusStart) + oRadiusEnd;	// [pixel]
	index	= round(radius*pixel_size/scan_step)*2;	// [increment = pixel*mm/pixel / (mm/increment)]
	slopeA2	= Lookup3D[index];		// deInclination
	slopeB2	= Lookup3D[index+1];	// zero-offset [mm]
	
	// Calculate 3D Point
	Point point;
	point.x	= (slopeB2-slopeB1)/(slopeA1-slopeA2);	// r [mm=(mm-mm)/(deInclination-deInclination)]
	point.y	= slopeA1*point.x + slopeB1;			// z [mm=mm*deInclination]
	return point;
}


Point Distance3DHelper::getDistance(double const rInner, double const rOuter) const
{
	Point	point;
	uint32_t	index;
	double	temp;
	double	slopeA1;
	double	slopeB1;
	double	slopeA2;
	double	slopeB2;
	
	/// Get inner slope [z=a*r+b]
	// Calculate radius
	temp		= rInner*pixel_size;
	temp		/= scan_step;
	index	= round(temp)*2.0;	// [increment = pixel * mm/pixel / (mm/increment)]
	if( index%2!=0 )
		index--;
	slopeA1	= Lookup3D[index];		// deInclination
	slopeB1	= Lookup3D[index+1];	// zero-offset [mm]
	
	/// Get outer slope [z=a*r+b]
	// Calculate radius
	temp		= rOuter*pixel_size;
	temp		/= scan_step;
	index	= round(temp)*2.0;	// [increment = pixel*mm/pixel / (mm/increment)]
	if( index%2!=0 )
		index--;
	slopeA2	= Lookup3D[index];		// deInclination
	slopeB2	= Lookup3D[index+1];	// zero-offset [mm]
	
	// Calculate 3D Point
	point.x	= (slopeB2-slopeB1)/(slopeA1-slopeA2);	// r [mm=(mm-mm)/(deInclination-deInclination)]
	point.y	= slopeA1*point.x + slopeB1;			// z [mm=mm*deInclination+mm]
	return point;
}


Point Distance3DHelper::getInnerDistance(double const rInner) const
{
	uint32_t	index;
	double	temp;
	double	slopeA1;
	double	slopeB1;
	double	slopeA2;
	double	slopeB2;
	
	/// Get inner slope [z=a*r+b]
	// Calculate radius
	temp		= rInner*pixel_size;
	temp		/= scan_step;
	index	= round(temp)*2.0;	// [increment = pixel * mm/pixel / (mm/increment)]
	if( index%2!=0 )
		index--;
	slopeA1	= Lookup3D[index];		// deInclination
	slopeB1	= Lookup3D[index+1];	// zero-offset [mm]
	
	/// Set the floor [z=a*r+b]
	// Calculate radius
	slopeA2	= 0;	// deInclination
	if(simulation)
		slopeB2	= -cameraHeight;	// zero-offset [mm]
	else
		slopeB2	= -cameraHeight;	// zero-offset [mm]
	
	// Calculate 3D Point
	Point point;
	point.x	= (slopeB2-slopeB1)/(slopeA1-slopeA2);	// r [mm=(mm-mm)/(deInclination-deInclination)]
	point.y	= slopeA1*point.x + slopeB1;		// z [mm=mm*deInclination+mm]
	return point;
}


Point Distance3DHelper::getOuterDistance(double const rOuter) const
{
	uint32_t	index;
	double	temp;
	double	slopeA1;
	double	slopeB1;
	double	slopeA2;
	double	slopeB2;
	
	/// Set the floor [z=a*r+b]
	// Calculate radius
	slopeA1	= 0;	// deInclination
	if(simulation)
		slopeB1	= -cameraHeight;	// zero-offset [mm]
	else
		slopeB1	= -cameraHeight;	// zero-offset [mm]
	
	/// Get outer slope [z=a*r+b]
	// Calculate radius
	temp		= rOuter*pixel_size;
	temp		/= scan_step;
	index	= round(temp)*2.0;	// [increment = pixel*mm/pixel / (mm/increment)]
	if( index%2!=0 )
		index--;
	slopeA2	= Lookup3D[index];		// deInclination
	slopeB2	= Lookup3D[index+1];	// zero-offset [mm]
	
	// Calculate 3D Point
	Point point;
	point.x	= (slopeB2-slopeB1)/(slopeA1-slopeA2);	// r [mm=(mm-mm)/(deInclination-deInclination)]
	point.y	= slopeA1*point.x + slopeB1;			// z [mm=mm*deInclination+mm]
	return point;
}


Distance3DHelper::~Distance3DHelper()
{
	std::cout << "Destructor of Distance3DHelper" << std::endl;
	
	if(Lookup3D != NULL)	delete[] Lookup3D;
}



