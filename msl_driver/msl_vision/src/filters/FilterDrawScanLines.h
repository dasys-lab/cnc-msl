/*
 * $Id: FilterDrawScanLines.h 1987 2007-04-09 16:58:10Z rreichle $
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
#ifndef FilterDrawScanLines_H
#define FilterDrawScanLines_H

#include "Filter.h"
#include "../global/Types.h"
#include "../helpers/ScanLineHelper.h"
#include "../helpers/ScanLineHelperDirected.h"

#include <SystemConfig.h>
#include <vector>


using namespace castor;

class FilterDrawScanLines  : public Filter {


	public:
		FilterDrawScanLines(int width, int height);
		~FilterDrawScanLines();
		
		unsigned char * process(unsigned char * src, unsigned int width, unsigned int height, ScanLineHelper & helper, bool gray);
		unsigned char * process(unsigned char * src, unsigned int width, unsigned int height, ScanLineHelperDirected & helper, bool gray);

	protected:

		SystemConfigPtr sc;

		void init();
		void cleanup();

		short negRanges[3][2];
		std::vector<Holder> addHolders;


};




#endif

