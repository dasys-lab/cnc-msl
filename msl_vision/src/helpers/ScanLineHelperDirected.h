/*
 * $Id: ScanLineHelperBall.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef ScanLineHelperDirected_H
#define ScanLineHelperDirected_H

#include <SystemConfig.h>

//#define HEIGHT 480
//#define WIDTH 640

#define MAXPOINTSDIRECTED 600

using namespace supplementary;

class ScanLineHelperDirected {


	public:
		
		ScanLineHelperDirected();
		ScanLineHelperDirected(int imWidth, int imHeight);
		~ScanLineHelperDirected();
	
		short * getLines();
		int * getLineOffsets();
		short getNumberLines();
		short getMaxPoints();


	protected:

		SystemConfig* sc;
		
		void init();
		int DrawLine(short * line, short ax, short ay, short ex, short ey);

		
		short mx;
		short my;
		short iRadius;
		short oRadius;
		short nLines;
		short maxPoints;

		short * lines;
		int * lineOffsets;

		int scWIDTH;
		int scHEIGHT;




};



#endif

