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
#ifndef PositionHelperDirected_H
#define PositionHelperDirected_H

#define HEIGHT 480
#define WIDTH 640
#include "../global/Types.h"
#include <SystemConfig.h>

using namespace supplementary;

class PositionHelperDirected{

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

		//Sony

		double LensBigAngle;
		double LensSmallAngle;
		double LensK1;

		double CenterDistance;
		double CenterHeight;
		double CameraHeight;
		double CameraMountedRotation;

		double MaxPixelXFromCenter;
		double MaxPixelYFromCenter;
	
		double DistanceToPixelPlane;



	public:
		static PositionHelperDirected * getInstance();
		static PositionHelperDirected * instance;
		PositionHelperDirected();
		Point3D getPointCam2Point3D(double x, double y, double r, double ball_r);
		Point3D getPointCam2Point3D(double x, double y, double r);

		Point3D getPointCam2Point3DSony(double x, double y, double r, double ball_r);
		Point3D getPointCam2Point3DSony(double x, double y, double r);
		Point getPoint3D2CamSony(double x, double y, double z);
		Point getPointCam2FieldSony(double x, double y);
		double getPoint3D2RadiusSony(double x, double y, double z);

		double LensCorrection(double v, double x, double y);
		Point3D NormalizePoint3D(Point3D p);
		Point3D CorrectAngle(Point3D p);
		Point3D CorrectAngleInv(Point3D p);


		Point getPointCam2Field(double x, double y);
		Point getPointCam2Angle(double x, double y);
		Point getPointField2Cam(double x, double y);
		Point getAngle2Cam(double x, double y);
		Point getPoint3D2Cam(double x, double y, double z);
		double getPoint3D2Radius(double x, double y, double z);
		double getPointCam2Dist(double x, double y);
};



#endif

