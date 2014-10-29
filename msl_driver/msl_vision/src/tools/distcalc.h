/*
 * $Id: distcalc.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef DISTCALC_H
#define DISTCALC_H


//#define CAMERAHOEHE 600
#define ITSTEPS 100
#define A 28.0950
#define B 23.4125
#define E 45.6654
#define R 30.0



class DistCalculator {

      public:
         DistCalculator() {CD=102.0;};
         double calcDistance(double ux, double uy, double *sx, double *sy);
		 void setHeightOfCam(int height);


      protected:
         double kx(double b);
		 double ky(double b);
		 double dkx(double b);
		 double dky(double b);
		 double gx(double a, double ux);
		 double gy(double a, double uy);
		 double nx(double b);
		 double ny(double b);
		 void solveNonLinear(double ux, double uy, double * a, double * b);
		 double distance(double b, double vx, double vy);
		 void spiegleVektor(double b, double ux, double uy, double * vx, double * vy);

		 int CAMERAHOEHE;
	public:
		 double CD;

   };


#endif

