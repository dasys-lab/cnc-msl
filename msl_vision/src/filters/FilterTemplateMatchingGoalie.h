/*
 * $Id: FilterSobelGradient.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef FilterTemplateMatchingGoalie_H
#define FilterTemplateMatchingGoalie_H

#include <vector>

#include "Filter.h"
#include "../helpers/PositionHelperDirected.h"
#include "../global/Types.h"
#include "../helpers/FootballField.h"

class  FilterTemplateMatchingGoalie : public Filter {


	public:
		FilterTemplateMatchingGoalie(int width, int height);
		~FilterTemplateMatchingGoalie();
		
		bool ballInKickerTest(unsigned char * src, int kickerNum);
		unsigned char * process(unsigned char * src, int* &ballb, int& ballCount, unsigned char * mask, std::vector<ROIData> &roiData, int maskThresh, int width, int height, int minRad, int maxRad,int threshold, unsigned char * gray);

	protected:
		int inline sign2(int s);
		int inline sign(int s);
		float inline dir(int gx, int gy, int threshold);
		int inline abs(int val);

		int *balls;
		int kickersPoints;

		static const int B_SIZE = 4;
		static const int MAXBALLNUM = 500;		
		static const int RADNUM = 50;
		static const float pi = 3.14159265;
		static const int CIRCPOINTS = 12;
		static const int COFFSET = 3*CIRCPOINTS;
		static const int Bx1 = 202, Bx2 = 233, Bx3 = 262, By1 = 247, By2 = 198, By3 = 246;

		void init(int width, int height);
		void cleanup();

		unsigned char * AreaLookup;
		int templateLookUp[RADNUM*CIRCPOINTS*3];
		int *YtoMinBallLookUp;
		int ballKickerPos[1000];

};




#endif

