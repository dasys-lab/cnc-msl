/*
 * $Id: FilterYUVToRGB.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef FilterHillClimbToSeg_H
#define FilterHillClimbToSeg_H

#include "Filter.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "../helpers/BalancePointHelper.h"
#include "../helpers/Lookuptable.h"

#include <SystemConfig.h>

#define MAX_VARIANZ 8000
//#define MAX_VARIANZ 600000

struct BlobInformation {
	unsigned short label;
	unsigned char color;
	int size;
};

struct BlobAbstract{
	unsigned short blobCount;
	unsigned int maxBlobSize;
	unsigned int pixelCount;
	unsigned color;
};

using namespace castor;

class FilterHillClimbToSeg : public Filter {


	public:
		FilterHillClimbToSeg(int width, int height);
		~FilterHillClimbToSeg();
		
		unsigned char* process(const unsigned char * src,unsigned char* segMap,Point* centroids, int centroidCount, unsigned int width, unsigned int height);
		//bool removeNoiseCluster(const int clusterNr, const int width, const int height);
		unsigned char* getRAW();
		unsigned char* getChrominanceInRegions();
		unsigned char* getOmnicamInRegions();
		unsigned char** getRoughColorRegions();
		

	protected:

		SystemConfigPtr sc;

		void init();
		void cleanup();

		void imLabel(unsigned char * in, unsigned short * out, int width, int height, std::vector<unsigned char> colors, std::vector<BlobInformation> & blobs, std::vector<BlobAbstract> &blobCount, const unsigned short min_blob_size);
		
	private:
		void colorizeImage(unsigned char* output, unsigned char* clusterToColor, int width, int height);
		unsigned char** roughColorRegions;
		unsigned char** realColors;
		unsigned char* raw;
		int iteration;
		bool debug;
		int m_width;
		int m_height;
		
		Lookuptable* oldLookupTable;
		
		//Debug Variablen
		unsigned char* ChrominanceInRegions;
		unsigned char* OmnicamInRegions;
		
		void buildYHistogram(const unsigned char* src,const unsigned char* raw, const unsigned char* clusterToColor, const int color, int* histogram, const int binSize,int radius);
		
		unsigned char findMaxInHist(const int* histogram, const int binSize);
		
		void checkSegmentationQuality( Lookuptable* oldTable, 
				 Lookuptable* newTable, 
				 Lookuptable* destTable,
				 const unsigned char* picture);
		
		int pixelPerColorOld[COLOR_COUNT];
		int pixelPerColorNew[COLOR_COUNT];
		
		std::vector<Point> m_linePix;
		
		void set_pixel(Point q);
		void bresenham_linie(Point P, Point q);
		
		bool isLinePix(Point q);		
	
		bool blueInitialized;
	

};




#endif

