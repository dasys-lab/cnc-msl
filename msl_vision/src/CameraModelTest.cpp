#include <stdio.h>
#include <stdlib.h>
#include "helpers/PositionHelperDirected.h"


int main(int argc, char * argv[]){

	PositionHelperDirected * posHelper = PositionHelperDirected::getInstance();

	
	Point p = posHelper->getPointCam2FieldSony(atof(argv[1]),atof(argv[2]));

	printf("p.x = %f p.y = %f\n", p.x, p.y);

	Point p2 = posHelper->getPoint3D2CamSony(p.x, p.y, 0.0);

	printf("CameraX = %f CameraY = %f\n", p2.x, p2.y);

	Point3D p3 = posHelper->getPointCam2Point3DSony(atof(argv[1]), atof(argv[2]), atof(argv[3]));

	printf("p3.x = %f p3.y = %f p3.z = %f\n", p3.x, p3.y, p3.z);

	double r = posHelper->getPoint3D2RadiusSony(p3.x, p3.y, p3.z);

	printf("radius: %f\n", r);

	return 0;
}



