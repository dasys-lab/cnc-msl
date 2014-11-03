/*
 * $Id: PositionHelperDirected.h 1531 2009-01-27 flseute $
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
#ifndef PositionHelper_H
#define PositionHelper_H

#define HEIGHT 480
#define WIDTH 640
#include "../global/Types.h"
#include <SystemConfig.h>


using namespace supplementary;

class PositionHelper {

	private:
		double abs_double(double d);

		double MX; //images size
		double MY;

		double fieldLength; //mm
		double fieldWidth; //mm

		double camProjDistX; //px
		double camProjDistY; //px
		double camZ; //mm
		double camX; //mm

		double dist2center; //px

		double ball_r; //mm

		double camAngle;


		int scWIDTH;
		int scHEIGHT;

		short mx;
		short my;
		int CameraZ;

		void goForLine(short ax, short ay, short ex, short ey, double angle, int * xcoord, int * ycoord);
		SystemConfig* sc;

		PositionHelper();
		static PositionHelper * instance;

	public:
		static PositionHelper * getInstance();

		Point3D getPointCam2Point3D(double x, double y, double r, double ball_r);
		Point3D getPointCam2Point3D(double x, double y, double r);

		Point getPointCam2Field(double x, double y);
		Point getPointCam2Angle(double x, double y);
		Point getPointField2Cam(double x, double y);
		Point getAngle2Cam(double x, double y);
		Point getPoint3D2Cam(double x, double y, double z);
		double getPoint3D2Radius(double x, double y, double z);

		Point3D getBallPositionFromBallMid(double x, double y);


};



#endif

