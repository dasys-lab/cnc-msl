/*
 * $Id: FilterExtractBlobs.h 2124 2007-04-14 17:01:41Z jewollen $
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
#ifndef FilterExtractBlobs_H
#define FilterExtractBlobs_H


#include "Filter.h"
#include "../global/Types.h"
#include "../helpers/DistanceLookupHelper.h"
#include <vector>

#define NREPS 10000


class FilterExtractBlobs  : public Filter {


	public:
		FilterExtractBlobs(int width, int height);
		~FilterExtractBlobs();
		
		unsigned char * process(unsigned char * src, unsigned int width, unsigned int height, std::vector<ROI>& retROIs, unsigned char blobColor, std::vector<BlobBounds> & blobs, DistanceLookupHelper & helper, int countThreshold = 800);

	protected:

		void init();
		void cleanup();

		ROI * rois;
		short * rep;
		int * count;

};




#endif

