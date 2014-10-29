/*
 * $Id: DistanceLookupHelper.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef DistanceLookupHelper_H
#define DistanceLookupHelper_H

#define HEIGHT 480
#define WIDTH 640

#define HLOOKUPSIZE 230

#define FILENAME "DistanceLookup.dat"

class DistanceLookupHelper{


	public:
		DistanceLookupHelper(int area);
		DistanceLookupHelper(short mx_, short my_);
		DistanceLookupHelper(char* filename, int area);
		DistanceLookupHelper(char* filename, int areaWidth, int areaHeight);
		DistanceLookupHelper(char* filename, short mx_, short my_);
		~DistanceLookupHelper();
	
		double * getLookupTable();
		int * getLookupTableInt();

		double * getHorizontalLookupTable();

		static DistanceLookupHelper * getCreatedInstance();

                int imWidth;
                int imHeight;

	protected:

		static DistanceLookupHelper * instance_;
		
		void init(char* name);
		void cleanup();

		double * LookupTable;
		int * LookupTableInt;

		double * HorizontalLookupTable;

		short mx;
		short my;
};



#endif

