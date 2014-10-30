/*
 * $Id: distcalc.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "distcalc.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>




/////////////////////////////////////////////////////////////////////////


double DistCalculator::kx(double b){


   //return (-64 + 103*cos(0.05*b));
   //return (b*(-0.945));
   //return (b);
   return (b);

};


double DistCalculator::ky(double b){

   //return (39+103*sin(0.05*b));
   //return (40.0 + b*0.326);
   //return (0.0085*b*b - 0.340*b + 26.376);
   //return (-0.0124*b*b + 1.191*b + (537.0 - 454.0));
   return (  sqrt(A*A * (1+b*b/(B*B))) + (CD - E)  );
};


double DistCalculator::dkx(double b){

   //return (-0.05*103*sin(0.05*b));
   //return (-0.945);
   //return (1);
   return (1);

};


double DistCalculator::dky(double b){

   //return (103*0.05*cos(0.05*b));
   //return (0.326);
   //return (2*0.0085*b - 0.340);
   //return (-2*0.0124*b + 1.191);
   return ( (1.0/(2.0*sqrt(A*A + A*A/(B*B)*b*b)))*(2.0*b*A*A/(B*B))   );
};


/////////////////////////////////////////////////////////////////////////////////////////////////


double DistCalculator::gx(double a, double ux){

   return a*ux;

};

double DistCalculator::gy(double a, double uy){

   return a*uy;
};


double DistCalculator::nx(double b){

   return dky(b);

};


double DistCalculator::ny(double b){

   return -dkx(b);

};

void DistCalculator::solveNonLinear(double ux, double uy, double * a, double * b){

   double ai, ai1, bi, bi1;
   ai = 0.0;
   bi = 0.0;

   for(int i = 0; i < ITSTEPS; i++){

      ai1 = ai - (gx(ai, ux) - kx(bi))*(-dky(bi)/(uy*dkx(bi)-ux*dky(bi))) - (gy(ai, uy) - ky(bi))*(dkx(bi)/(uy*dkx(bi)-ux*dky(bi)));
      bi1 = bi - (gx(ai, ux) - kx(bi))*(-uy/(uy*dkx(bi)-ux*dky(bi))) - (gy(ai, uy) - ky(bi))*(ux/(uy*dkx(bi)-ux*dky(bi)));
      ai = ai1;
      bi = bi1;

   };

   *a = ai;
   *b = bi;


};


double DistCalculator::distance(double b, double vx, double vy){

   double erg = 0.0;
   
   if (fabs(vx) > 1.0E-8)
	erg = kx(b) - (ky(b) + CAMERAHOEHE)*(vx/vy);

   return erg;


};

void DistCalculator::spiegleVektor(double b, double ux, double uy, double * vx, double * vy){

   double alphaN = atan2(ny(b), nx(b));
   double alphaS = atan2(-uy, -ux);

   double rotAlpha = 2*(alphaN - alphaS);

   /*printf("U: %f %f\n", ux, uy);
   printf("Ableitung: %f %f\n", dkx(b), dky(b));
   printf("Normale: %f %f\n", nx(b), ny(b));
   printf("AlphaS: %f\n", alphaS);
   printf("AlphaN: %f\n", alphaN);*/

   *vx = -cos(rotAlpha)*ux + sin(rotAlpha)*uy;
   *vy = -sin(rotAlpha)*ux - cos(rotAlpha)*uy;

   /*printf("RotAlpha: %f\n", rotAlpha);

   printf("V: %f %f\n", *vx, *vy);*/

};



double DistCalculator::calcDistance(double ux, double uy, double *sx, double *sy){


   double a, b;

   solveNonLinear(ux, uy, &a, &b);
   *sx = a*ux;
   *sy = a*uy;

   double vx, vy;
   spiegleVektor(*sx, ux, uy, &vx, &vy);

   //printf("%f %f\n", *sx, *sy);
   
   double d = distance(*sx, vx, vy);
   //printf("%f\n", d);

   return d;


};


void DistCalculator::setHeightOfCam(int height){


	CAMERAHOEHE = height;


};


