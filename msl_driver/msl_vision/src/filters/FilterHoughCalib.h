/*
 * $Id: FilterHoughCalib.h 1987 2007-04-09 16:58:10Z rreichle $
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
#ifndef FilterHoughCalib_H
#define FilterHoughCalib_H

#include "Filter.h"


class FilterHoughCalib  : public Filter {


	public:
		FilterHoughCalib(int width, int height);
		~FilterHoughCalib();
		
		unsigned char * process(unsigned char * src, int width, int height);
		void DrawCircle(int midX, int midY, int rad, unsigned char * space, int width, int height);

	protected:

		void init();
		void cleanup();

		int mx;
		int my;
		int radius;

		void DrawCircleInc(int midX, int midY, int rad, int * space, int width, int height);
		

};




#endif

