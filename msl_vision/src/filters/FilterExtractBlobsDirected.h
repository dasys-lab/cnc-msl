/*
 * $Id: FilterExtractBlobsDirected.h 2124 2007-04-14 17:01:41Z jewollen $
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
#ifndef FilterExtractBlobsDirected_H
#define FilterExtractBlobsDirected_H


#include "Filter.h"
#include "../helpers/DistanceLookupHelper.h"
#include "../global/Types.h"
#include <vector>

#define NREPS 10000



class FilterExtractBlobsDirected  : public Filter {


	public:
		FilterExtractBlobsDirected(int width, int height);
		~FilterExtractBlobsDirected();
		
		unsigned char * process(unsigned char * src, unsigned int width, unsigned int height, std::vector<ROI>& retROIs, unsigned char blobColor, std::vector<BlobBounds> & blobs, int countThreshold = 800);

	protected:

		void init();
		void cleanup();

		ROI * rois;
		short * rep;
		int * count;

};




#endif

