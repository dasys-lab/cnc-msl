/*
 * $Id: ScanLineHelper.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef ScanLineHelper_H
#define ScanLineHelper_H

#include <SystemConfig.h>

//#define HEIGHT 480
//#define WIDTH 640

#define MAXPOINTS 350

using namespace supplementary;

class ScanLineHelper{


	public:
		
		ScanLineHelper();
		ScanLineHelper(int imWidth, int imHeight);
		~ScanLineHelper();
	
		short * getInnerLines();
		short * getInnerLinesN();
		short * getOuterLines();
		short * getOuterLinesN();
		short * getCircles();
		short * getCircleOffsets();
		short getNumberLines();
		short getNumberCircles();
		short getMaxPoints();


	protected:

		SystemConfig* sc;
		
		void init();
		void DrawLine(short * line, short * nPoints, short ax, short ay, short ex, short ey);
		void initCircles();

        int ownId;
		
		short mx;
		short my;
		short iRadius;
		short oRadius;
		short frontScanlineDistance;
		short nLines;
		short maxPoints;

		short * innerLines;
		short * innerLinesN;
		short * outerLines;
		short * outerLinesN;
		short * circles;
		short * circleOffsets;
		short numberOfCircles;

		int scWIDTH;
		int scHEIGHT;




};



#endif

