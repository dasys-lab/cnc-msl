/*
 * $Id: FilterYUVToRGB.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "FilterHillClimbToSeg.h"

#include <algorithm>
#include <functional>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <sys/time.h>
#include "../helpers/BalancePointHelper.h"
#include "../helpers/ConnectedComponents.h"
#include "FilterBitmapWriter.h"
#include "../helpers/Lookuptable.h"
#include "../helpers/HistogramHelper.h"
#include "../filters/FilterYUVToSeg.h"
#include "../helpers/SharedMemoryHelper.h"

#define MIN_BLOB_SIZE 15

FilterHillClimbToSeg::FilterHillClimbToSeg(int width, int height):Filter(OF_RGB, width, height), sc() {

	this->sc = SystemConfig::getInstance();

	debug = 1;
	
	if (debug) {
		ChrominanceInRegions = new unsigned char[256*256];
		OmnicamInRegions = new unsigned char[width*height];
	}
		
	
	init();	
	
	blueInitialized = false;
	
	SharedMemoryHelper sharedMemory;

	oldLookupTable = NULL;
	oldLookupTable = new Lookuptable;
	memcpy(oldLookupTable,sharedMemory.readLookupTable(),sizeof(Lookuptable));			
	/*
			new Lookuptable;
	//memset(oldLookupTable,0,sizeof(Lookuptable));
	printf("pointer To oldLookupTable: %p\n",oldLookupTable);
	
	for (int i = 0; i < LOOKUPTABLE_SIZE; i++) 
	{
		for(int j = 0; j < LOOKUPTABLE_SIZE; j++) 
		{
			oldLookupTable->table[i][j] = COLOR_UDEF;
		}
	}
	
	for (int i = 0; i < COLOR_COUNT; i++) 
	{
		oldLookupTable->limits[i].low = 0;
		oldLookupTable->limits[i].high = 255;
	}
	*/
	raw = new unsigned char[width*height];
	iteration = 0;

	Configuration *vision = (*this->sc)["Vision"];
	
	printf("YELLOW: %d\n", vision->get<int>("Vision", "Calibration", "YELLOW_U_LOW", NULL));
	printf("YELLOW: %d\n", vision->get<int>("Vision", "Calibration", "YELLOW_U_HIGH", NULL));
	printf("YELLOW: %d\n", vision->get<int>("Vision", "Calibration", "YELLOW_V_LOW", NULL));
	printf("YELLOW: %d\n", vision->get<int>("Vision", "Calibration", "YELLOW_V_HIGH", NULL));
	 
	roughColorRegions = new unsigned char*[COLOR_COUNT];
	for (int i = 0; i <COLOR_COUNT; i++) {
		roughColorRegions[i] = new unsigned char[4];
		memset(roughColorRegions[i],0,sizeof(char)*4);
	}
	
	realColors = new unsigned char*[COLOR_COUNT];
	for (int i = 0; i <COLOR_COUNT; i++) {
		realColors[i] = new unsigned char[3];
		memset(realColors[i],0,3);
	}
	
	realColors[COLOR_RED][0] = 255;
	realColors[COLOR_GREEN][1] =255;
	realColors[COLOR_BLUE][2] =255;
	realColors[COLOR_YELLOW][0] =255;
	realColors[COLOR_YELLOW][1] =255;
	

	
	roughColorRegions[COLOR_YELLOW][0] = vision->get<unsigned char>("Vision", "Calibration", "YELLOW_U_LOW", NULL);
	roughColorRegions[COLOR_YELLOW][1] = vision->get<unsigned char>("Vision", "Calibration", "YELLOW_U_HIGH", NULL);
	roughColorRegions[COLOR_YELLOW][2] = vision->get<unsigned char>("Vision", "Calibration", "YELLOW_V_LOW", NULL);
	roughColorRegions[COLOR_YELLOW][3] = vision->get<unsigned char>("Vision", "Calibration", "YELLOW_V_HIGH", NULL);

	roughColorRegions[COLOR_RED][0] = vision->get<unsigned char>("Vision", "Calibration", "RED_U_LOW", NULL);
	roughColorRegions[COLOR_RED][1] = vision->get<unsigned char>("Vision", "Calibration", "RED_U_HIGH", NULL);
	roughColorRegions[COLOR_RED][2] = vision->get<unsigned char>("Vision", "Calibration", "RED_V_LOW", NULL);
	roughColorRegions[COLOR_RED][3] = vision->get<unsigned char>("Vision", "Calibration", "RED_V_HIGH", NULL);

	roughColorRegions[COLOR_BLUE][0] = vision->get<unsigned char>("Vision", "Calibration", "BLUE_U_LOW", NULL);
	roughColorRegions[COLOR_BLUE][1] = vision->get<unsigned char>("Vision", "Calibration", "BLUE_U_HIGH", NULL);
	roughColorRegions[COLOR_BLUE][2] = vision->get<unsigned char>("Vision", "Calibration", "BLUE_V_LOW", NULL);
	roughColorRegions[COLOR_BLUE][3] = vision->get<unsigned char>("Vision", "Calibration", "BLUE_V_HIGH", NULL);

	roughColorRegions[COLOR_GREEN][0] = vision->get<unsigned char>("Vision", "Calibration", "GREEN_U_LOW", NULL);
	roughColorRegions[COLOR_GREEN][1] = vision->get<unsigned char>("Vision", "Calibration", "GREEN_U_HIGH", NULL);
	roughColorRegions[COLOR_GREEN][2] = vision->get<unsigned char>("Vision", "Calibration", "GREEN_V_LOW", NULL);
	roughColorRegions[COLOR_GREEN][3] = vision->get<unsigned char>("Vision", "Calibration", "GREEN_V_HIGH", NULL);

	roughColorRegions[COLOR_BLACK][0] = vision->get<unsigned char>("Vision", "Calibration", "BLACK_U_LOW", NULL);
	roughColorRegions[COLOR_BLACK][1] = vision->get<unsigned char>("Vision", "Calibration", "BLACK_U_HIGH", NULL);
	roughColorRegions[COLOR_BLACK][2] = vision->get<unsigned char>("Vision", "Calibration", "BLACK_V_LOW", NULL);
	roughColorRegions[COLOR_BLACK][3] = vision->get<unsigned char>("Vision", "Calibration", "BLACK_V_HIGH", NULL);
}


FilterHillClimbToSeg::~FilterHillClimbToSeg(){

	cleanup();

}
		
unsigned char* FilterHillClimbToSeg::getOmnicamInRegions() {
	return OmnicamInRegions;
}

unsigned char* FilterHillClimbToSeg::getChrominanceInRegions() {
	return ChrominanceInRegions;
}



unsigned char* FilterHillClimbToSeg::process(const unsigned char * src,unsigned char* segMap,Point* centroids, int centroidCount, unsigned int width, unsigned int height){
	
	m_height = height;
	m_width = width;
	
	Lookuptable newLookupTable;
	Lookuptable destTable;
	
	// Index Map shows an cluster belongs to a color
	unsigned char clusterToColor[centroidCount];
	// Index Map shows which cluster belongs to which color.
	unsigned char realColors[COLOR_COUNT][3] = {{255,0,255},{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,0,0}};	
	
			
	//Segmentiere das Bild mit Hilfe der Clustererkennung

	int u = 0;
	int v = 0;
	int y = 0;
	float h_ = 0;
	float s_ = 0;
	float v_ = 0;
	
	unsigned char * src_ptr =const_cast<unsigned char* >(src);
	unsigned char * raw_p = raw;
	float * hsv = new float[width*height*3];
	float* hsv_ptr = hsv;
	
	for (int i = 0; i<COLOR_COUNT; i++) {
		newLookupTable.limits[i].low = 0;
		newLookupTable.limits[i].high = 255;	
	}
	
	
	
	for(unsigned int i = 0; i<width*height/2; i++) {

		u = (*src_ptr++);
		y = (*src_ptr++);
		v = (*src_ptr++);
		(*raw_p++) = segMap[u*256+v];
		HistogramHelper::YUVtoHSV(y,u,v, h_,s_,v_);
		(*hsv_ptr++) = h_;
		(*hsv_ptr++) = s_;
		(*hsv_ptr++) = v_;
		
		y = *(src_ptr++);
		(*raw_p++) = segMap[u*256+v];
		HistogramHelper::YUVtoHSV(y,u,v, h_,s_,v_);
		(*hsv_ptr++) = h_;
		(*hsv_ptr++) = s_;
		(*hsv_ptr++) = v_;
	
	}

	memset(clusterToColor,0, centroidCount*sizeof(char));
	
	//Find alle Cluster, die zu einer Farbe, also im roughColorRegions Bereich liegen
	//und schreibe diese in colorToRegions und in realColors
	for (int i= 0; i< centroidCount; i++) {
		int x = centroids[i].x;
		int y = centroids[i].y;
		
		for (int j = 0; j<COLOR_COUNT; j++) {
			//Yellow behandlung:
			/*
			if (j == COLOR_YELLOW) {
				Point right_top;
				Point left_bottom;
				
				right_top.x = roughColorRegions[j][1];
				right_top.y = roughColorRegions[j][2];
				
				left_bottom.x = roughColorRegions[j][0];
				left_bottom.y = roughColorRegions[j][3];
				
				bresenham_linie(right_top, left_bottom);
				
				
				for (int l_y = roughColorRegions[j][2]; l_y<roughColorRegions[j][3]; l_y++) {
					for (int l_x = roughColorRegions[j][0]; l_x< roughColorRegions[j][1]; l_x++) {
						Point p; 
						p.x = l_x;
						p.y = l_y;
						if (isLinePix(p)) {					
							break;
						} else if (x == l_x && y == l_y) {
								clusterToColor[i] = j; 
								printf("YELLOW: clusterNr: %d\n", i);
								//break;
						} 
						
						
						
					}
					
				}
		} */ 
			if (0) {
			}
			else 
			{
				if (x>roughColorRegions[j][0] && x<roughColorRegions[j][1] && y>roughColorRegions[j][2] && y<roughColorRegions[j][3]) 
				{							
					clusterToColor[i] = j;			
				}
			}
		}
	}
	
	//Bild Histogramm über die Y Werte:
	const int binSize = 25;
	int histogramBlue[binSize];
	buildYHistogram(src, raw, clusterToColor, COLOR_BLUE, histogramBlue, binSize, 0);
	unsigned char maxBinBlue = findMaxInHist(histogramBlue,binSize);
	
	newLookupTable.limits[COLOR_BLUE].low = 0;
	newLookupTable.limits[COLOR_BLUE].high =maxBinBlue*(255/binSize);

	if (newLookupTable.limits[COLOR_BLUE].high > 40) {
		newLookupTable.limits[COLOR_BLUE].high = 40;
	}

	printf("Blue high limit: %d\n",newLookupTable.limits[COLOR_BLUE].high);

	if (debug) {		
		memset(ChrominanceInRegions, 0, 256*256);
		for (int i = 0; i<256; i++) {
			for (int j = 0; j<256; j++) {
				if (clusterToColor[raw[i*256+j]] == 0) {
					ChrominanceInRegions[i*256+j] = raw[i*256+j]+50;
				}
			}
		}
			
		memset(OmnicamInRegions,0,width*height);
		for (int i = 0; i< height; i++) {
			for (int j = 0; j<width; j++) {
				if ( clusterToColor[ raw[i*width + j ]] )
					OmnicamInRegions[i*width + j] = raw[i*width + j]+50;
			}
		}
	}
	
												
	//Berechne Schwerpunkte der OmnicamCluster
	Point centroidsOmnicam[centroidCount];	
	long xVal[centroidCount] ;
	long yVal[centroidCount] ;
	int balancePointsCount[centroidCount];
	long varianz[centroidCount];
	
	memset(centroidsOmnicam,0,centroidCount*sizeof(Point));
	memset(xVal,0,centroidCount*sizeof(long));
	memset(yVal,0,centroidCount*sizeof(long));
	memset(varianz,0,centroidCount*sizeof(long));
	memset(balancePointsCount,0,centroidCount*sizeof(int));

	unsigned char * raw_ptr = raw;	

	
	for (unsigned int i = 0; i< height; i++) {
		for (unsigned int j = 0; j< width; j++) {
			if (hsv[(i*width+j)*3]>0.3) {
				if (clusterToColor[*raw_ptr]) {
					xVal[*raw_ptr] += i;
					yVal[*raw_ptr] += j;
					balancePointsCount[*raw_ptr]++;
				}
			}
			raw_ptr++;
		}
	}
	
	delete[] hsv;
	
	// Schwerpunkt berechnen
	for (int i = 0; i< centroidCount; i++) {
		if (balancePointsCount[i]) {
			centroidsOmnicam[i].x = xVal[i] / balancePointsCount[i];
			centroidsOmnicam[i].y = yVal[i] / balancePointsCount[i];			
		}
	}

	// Varianz der Cluster berechnen
	for(unsigned int i = 0; i< height; i++) {
		for(unsigned int j = 0; j< width; j++) {
			int index = raw[i*width+j];			
				varianz[index] += (centroidsOmnicam[index].x-i)*(centroidsOmnicam[index].x-i);
				varianz[index] += (centroidsOmnicam[index].y-j)*(centroidsOmnicam[index].y-j);								
		}
	}	
	
	for (int i = 0 ; i < centroidCount; i++) 
	{
		if (balancePointsCount[i])
			varianz[i] = varianz[i]/ balancePointsCount[i];
		
		if (clusterToColor[i] == COLOR_YELLOW)
			printf("Color: %d,blob:%d Varianz: %d\n",clusterToColor[i],i,varianz[i]);
	}
	
	
	//entferne Cluster, deren Varianz zu groß ist	roughColorRegions
	
	unsigned short * tgt = new unsigned short[width*height];
	std::vector<BlobInformation> blobs;
	std::vector<unsigned char> colors;
	std::vector<BlobAbstract> blobCount;
	colors.clear();		
	
	// Finde cluster die über der, der Varianzschwelle liegen.
	// Schwarz soll unabhaengig von der Varianz immer bestehen bleiben
	for (int i = 0; i< centroidCount; i++) {		
		colors.push_back(i);	
		BlobAbstract blobInfo;				
		memset(&blobInfo,0,sizeof(BlobAbstract));
		blobInfo.color = i;
		blobCount.push_back(blobInfo);							
	}
	//
	imLabel(raw,tgt,width,height,colors,blobs,blobCount, MIN_BLOB_SIZE);
	
	delete[] tgt;
	
	//suche den größten Cluster und entferne ihne: Dat is naemlich dat schwarz!
	int maxCluster = 0;
	int maxClusterNumber = 0;
	for (int i = 0; i < blobCount.size(); i++) 
	{
		if (blobCount[i].pixelCount > maxCluster) 
		{
			maxCluster = blobCount[i].pixelCount;
			maxClusterNumber = i;
		}			
	}
	//printf("Allergroesste cluster: %d\n",maxClusterNumber);
	clusterToColor[maxClusterNumber] = COLOR_BLACK;
	
	int histogramBlack[binSize];
	buildYHistogram(src,raw,clusterToColor, COLOR_BLACK, histogramBlack, binSize,253);
	unsigned char maxBinBlack = findMaxInHist(histogramBlack,binSize);
	
	newLookupTable.limits[COLOR_BLACK].low = 8;
	newLookupTable.limits[COLOR_BLACK].high =35;// maxBinBlack*(255/binSize);
		
	for (int i = 0; i <blobCount.size(); i++) {
		BlobAbstract info = blobCount[i];
		if (clusterToColor[info.color] == COLOR_YELLOW) {
			printf("Color: %d,blob:%d, maxBlobSize %d, pixelCount %d ",info.color,info.blobCount, info.maxBlobSize, info.pixelCount);
			printf("Varianz %d\n", varianz[info.color]);
		}
		
		
		//printf("Color: %d, maxBlobSize %d, pixelCount %d\n",info.color, info.maxBlobSize, info.pixelCount);
		if (clusterToColor[info.color] == COLOR_BLACK ||  varianz[i] < MAX_VARIANZ || clusterToColor[info.color] == COLOR_RED || clusterToColor[info.color] == COLOR_BLUE) {
			;
		} else {
			if (clusterToColor[info.color] == COLOR_YELLOW) 
			{
				if (info.blobCount >20) {
					clusterToColor[info.color] =0 ;
				}
			} else {			
				if ((info.blobCount>8 && ((float)info.maxBlobSize/(float)info.pixelCount)<0.4) || info.blobCount == 0 || info.pixelCount >80000) {			
					clusterToColor[info.color] =0;
				}	
			}
		}		
	}
	

	newLookupTable.limits[COLOR_BLACK].low = 0;
	newLookupTable.limits[COLOR_BLACK].high = 35;
	
	for(int i = LOOKUPTABLE_OFFSET; i< LOOKUPTABLE_OFFSET+LOOKUPTABLE_SIZE; i++) {
		for (int j = LOOKUPTABLE_OFFSET; j<LOOKUPTABLE_OFFSET+LOOKUPTABLE_SIZE; j++) {			
			newLookupTable.table[i-LOOKUPTABLE_OFFSET][j-LOOKUPTABLE_OFFSET]  = 0;
			if (clusterToColor[segMap[i*256+j]] != COLOR_UDEF) {
				newLookupTable.table[i-LOOKUPTABLE_OFFSET][j-LOOKUPTABLE_OFFSET] = clusterToColor[segMap[i*256+j]];			
				if (clusterToColor[segMap[i*256+j]] == COLOR_RED && i+2<LOOKUPTABLE_SIZE && j+2<LOOKUPTABLE_SIZE)
				{
					for (int k = -2; k <=2; k++) {
						for (int l = -2; l<=2; l++) {
							newLookupTable.table[i-LOOKUPTABLE_OFFSET+k][j-LOOKUPTABLE_OFFSET+l] = COLOR_RED;
						}
					}
				}				
			}	
		}		
	}	
	
	//Vergleiche neue mit aler Kalibrierung und merge die Lookuptables
	memset(&destTable,0,sizeof(Lookuptable));
	
	if (oldLookupTable == NULL) {
		oldLookupTable = new Lookuptable;
		memcpy(&destTable,&newLookupTable,sizeof(Lookuptable));
		memcpy(oldLookupTable,&destTable,sizeof(Lookuptable));
	} else {
		checkSegmentationQuality(oldLookupTable,&newLookupTable, &destTable,src);
		memcpy(oldLookupTable,&destTable,sizeof(Lookuptable));		
	}
	
	SharedMemoryHelper sharedMemory;
	sharedMemory.writeLookupTable(&destTable);	
	
	
	if (debug) {	
		colorizeImage(outputBuffer,(unsigned char*)clusterToColor,640,480);
	}																					
															
	return outputBuffer;
}

void FilterHillClimbToSeg::init(){

}

void FilterHillClimbToSeg::buildYHistogram(const unsigned char* src,const unsigned char* raw, const unsigned char* clusterToColor, const int color, int* histogram, const int binSize, int radius = 0) {
	
	const int binWidth = 255/(binSize-1);
	const int cx = 340;
	const int cy = 233;
	
	for (int i = 0; i<binSize; i++) 
	{
		histogram[i]=0;
	}
	for (int  i = 0; i< m_width; i++) 
	{		
		for (int j = 0; j < m_height; j++) 
		{
			if (clusterToColor[raw[(i*m_height)+j]] == color) 
			{			
				if (radius != 0 || (((i-cx)*(i-cx))+((j-cy)*(j-cy)))<(radius*radius)) 
				{
					histogram[(src[((i*m_height)+j)*3]/binWidth)]++;
				}
				else if (radius == 0){
					histogram[(src[(i*m_height+j)*3]/binWidth)]++;
				}				
			}	
		}
		
	}
}

void FilterHillClimbToSeg::colorizeImage(unsigned char* output, unsigned char* clusterToColor, int width, int height) {
	
	memset(output,0,width*height*3*sizeof(char));	
	for (int i = 0; i< width*height; i++) {			
		int clusterNr = raw[i];	
		int color = clusterToColor[clusterNr];		
		output[(i)*3] = 255;					
						
		output[((i)*3)+1] = 255;										
		output[((i)*3)+2] = 255;		
		if (clusterToColor[clusterNr] != 0) {					
			output[(i)*3] 		= realColors[color][0];										
			output[((i)*3)+1] 	= realColors[color][1];										
			output[((i)*3)+2] 	= realColors[color][2];
		} 
	}
}

unsigned char FilterHillClimbToSeg::findMaxInHist(const int* histogram, const int binSize)
{
	int delta = 1;
	unsigned char binCounter = 0;
	bool maxFound = false;
	
	while (delta > 0 && binCounter < binSize-1) 
	{
		delta = histogram[binCounter+1] - histogram[binCounter];
		binCounter++;
	}
	
	delta = -1;
	while (delta < 0 && binCounter < binSize -1) 
	{
		binCounter++;
		delta = histogram[binCounter+1] - histogram[binCounter];
	}
	
	return binCounter;
}

unsigned char* FilterHillClimbToSeg::getRAW() {
	return raw;
}

void FilterHillClimbToSeg::cleanup(){
	delete[] raw;
	delete oldLookupTable;
	if (debug) 
	{
		delete[] ChrominanceInRegions;
		delete[] OmnicamInRegions;
	}
}

unsigned char** FilterHillClimbToSeg::getRoughColorRegions() {
	return roughColorRegions;
}

void FilterHillClimbToSeg::checkSegmentationQuality(Lookuptable* oldTable, 
													Lookuptable* newTable,
													Lookuptable* destTable, 
													const unsigned char* picture) 
{
	FilterYUVToSeg filterSegOld(m_width, m_height);
	FilterYUVToSeg filterSegNew(m_width, m_height);
	
	std::vector<BlobInformation> blobsOld;
	std::vector<unsigned char> colorsOld;
	std::vector<BlobAbstract> blobCountOld;
	colorsOld.clear();		
	
	unsigned char* oldSegmentation;
	unsigned char* newSegmentation;
	unsigned short tgt[m_width*m_height];
	
	memset(pixelPerColorOld,0,sizeof(int)*COLOR_COUNT);
	memset(pixelPerColorNew,0,sizeof(int)*COLOR_COUNT);
	// Segmentiere das Bild mit der alten Tabellema
	
	oldSegmentation = filterSegOld.process(const_cast<unsigned char* >(picture),oldTable, m_width*m_height*2);
	newSegmentation = filterSegNew.process(const_cast<unsigned char* >(picture),newTable, m_width*m_height*2);
	
	// label die alte Segmentierung
	for (int i = COLOR_UDEF; i< COLOR_COUNT; i++) {		
		colorsOld.push_back(i);	
		BlobAbstract blobInfo;				
		memset(&blobInfo,0,sizeof(BlobAbstract));
		blobInfo.color = i;
		blobCountOld.push_back(blobInfo);							
	}
	
	imLabel(oldSegmentation,tgt,m_width,m_height,colorsOld,blobsOld,blobCountOld, 10);
	
	// label die neue Segmentierung
	std::vector<BlobInformation> blobsNew;
	std::vector<unsigned char> colorsNew;
	std::vector<BlobAbstract> blobCountNew;
	colorsNew.clear();		
	
	for (int i = COLOR_UDEF; i< COLOR_COUNT; i++) {		
		colorsNew.push_back(i);	
		BlobAbstract blobInfo;				
		memset(&blobInfo,0,sizeof(BlobAbstract));
		blobInfo.color = i;
		blobCountNew.push_back(blobInfo);							
	}
	imLabel(newSegmentation,tgt,m_width,m_height,colorsNew,blobsNew,blobCountNew, 10);
	
	//Errechne die gesamt markierten Pixel pro Farbe
	for(int i = 0; i< blobsNew.size() ; i++)
	{		
		pixelPerColorNew[blobsNew[i].color] += blobsNew[i].size;
	}
	
	for(int i = 0; i< blobsOld.size() ; i++)
	{
		pixelPerColorOld[blobsOld[i].color] += blobsOld[i].size;
	}
	
	
	memset(destTable,0,sizeof(Lookuptable));
				
	for (int i = COLOR_UDEF+1; i < COLOR_COUNT; i++) 
	{
		
		int maxBlobSizeNew = blobCountNew[i].maxBlobSize;
		int pixelPerColorNewV = pixelPerColorNew[i];
		
		int maxBlobSizeOld = blobCountOld[i].maxBlobSize;
		int pixelPerColorOldV = pixelPerColorOld[i];
		
		
		//Wenn bei beiden kalibrierungen keine roten pixel erkannt wurden,
		//dann behalte die alte. 
		
		bool takeNew = false;
		
		if((blobCountNew[i].maxBlobSize*1.0 / pixelPerColorNew[i]*1.0) >= 
				  (blobCountOld[i].maxBlobSize*1.0 / pixelPerColorOld[i]*1.0) || i == COLOR_BLACK)
		{
			takeNew = true;
		}
		
		if (blobCountNew[i].blobCount == 0 && blobCountOld[i].blobCount>0) {
			takeNew = false;			
		}
		
		if (blobCountNew[i].blobCount > 0 && (blobCountNew[i].pixelCount*1.0/blobCountOld[i].pixelCount*1.0)<0.5) {
			takeNew = false;
		}

		if (blobCountNew[i].color == COLOR_BLUE && blobCountNew[i].maxBlobSize > 2000 || blueInitialized == false) {
			if (blobCountNew[i].maxBlobSize>2000)
				blueInitialized = true;
			takeNew = true;
		}
		
		
		if (takeNew) {
			for (int j = 0; j < LOOKUPTABLE_SIZE; j++) {
				for (int k = 0; k < LOOKUPTABLE_SIZE; k++) {
					if (newTable->table[j][k] == i && destTable->table[j][k]==0)
						destTable->table[j][k] =(unsigned char)i;
				}
			}
			printf("Color: %d, neu!\n",i);
			destTable->limits[i].low = newTable->limits[i].low;
			destTable->limits[i].high = newTable->limits[i].high;
		} else {
			for (int j = 0; j < LOOKUPTABLE_SIZE; j++) {
				for (int k = 0; k < LOOKUPTABLE_SIZE; k++) {
					if (oldTable->table[j][k] == i && destTable->table[j][k]==0)
						destTable->table[j][k] =(unsigned char) i;
				}
			}
			printf("Color: %d, alt!\n",i);
			destTable->limits[i].low = oldTable->limits[i].low;
			destTable->limits[i].high = oldTable->limits[i].high;
		}
	}
		
		

}


void FilterHillClimbToSeg::imLabel(unsigned char * in, unsigned short * out, int width, int height, std::vector<unsigned char> colors, std::vector<BlobInformation> & blobs, std::vector<BlobAbstract> &blobCount, const unsigned short treshold){

	unsigned char valid[256];
	memset(valid,0,256);
	memset(out,0,width*height*sizeof(unsigned short));

	for(unsigned int i = 0; i < colors.size(); i++){
		//printf("Colors[i] = %d\n", colors[i]);
		valid[colors[i]] = 1;
		if(colors[i] == 0)
			printf("Valid Color = 0!!!!!!\n");

	}

	unsigned char * tmp = in;
	unsigned short * reps = new unsigned short[60000];
	memset(reps, 0, 60000*sizeof(unsigned short));
	//printf("reps[0]: %d\n", reps[0]);
	unsigned short * repColors = new unsigned short[60000];
	int * blobSizes = new int[60000];

	unsigned short numberOfReps = 0;

	unsigned int counter = 0;

	for(int j = 0; j < width; j++){
		tmp[j] = 0;
	}

	for(int i = 0; i < height; i++){
		tmp[i*width] = 0;
	}


	for(int i = 1; i < height; i++){
		for(int j = 1; j < width - 1; j++){

			unsigned char currColor = tmp[i*width + j];

			if(valid[currColor] > 0){

				
				unsigned char left = tmp[i*width + j - 1];
				unsigned char top = tmp[(i-1)*width + j];

				unsigned short leftLabel = out[i*width + j - 1];
				unsigned short topLabel = out[(i-1)*width + j];

				unsigned char topLeft = tmp[(i-1)*width + j - 1];
				unsigned short topLeftLabel = out[(i-1)*width + j - 1];

				unsigned char topRight = tmp[(i-1)*width + j + 1];
				unsigned short topRightLabel = out[(i-1)*width + j + 1];

				if(currColor == left){

					unsigned short leftRep = reps[leftLabel];
	
					counter = 0;
					while(reps[leftRep] != leftRep){
						leftRep = reps[leftRep];
						counter++;
						if(counter > 100000){
							printf("Kraut1");
							exit(1);
						}

					}

					if(currColor != top){

						if(currColor != topRight){

							out[i*width + j] = leftLabel;
							blobSizes[leftRep]++;

						}
						else {

							unsigned short topRightRep = reps[topRightLabel];
	
							counter = 0;
							while(reps[topRightRep] != topRightRep){
								topRightRep = reps[topRightRep];
								counter++;
								if(counter > 100000 - 5){
									printf("rep: %d reps[reps]: %d Label: %d\n", topRightRep, reps[topRightRep], topRightLabel);
			
								}
								if(counter > 100000){
									printf("Indices i: %d j: %d\n", i, j);
									printf("NumberOfReps: %d\n", numberOfReps);
									printf("Kraut2");
									exit(1);
								}
		
							}


							
							if(leftRep == topRightRep){

								out[i*width + j] = leftLabel;
								blobSizes[leftRep]++;

							}
							else {

								//printf("Cluster Fusion!!!!!!!!!!!!!!!!\n");

								if(leftRep < topRightRep){
									out[i*width + j] = leftLabel;

									reps[topRightRep] = leftRep;

									blobSizes[leftRep] += blobSizes[topRightRep] + 1;
	
								}
								else {

									out[i*width + j] = leftLabel;

									reps[leftRep] = topRightRep;

									blobSizes[topRightRep] += blobSizes[leftRep] + 1;

								}

							}


						}
						//printf("currColor != top for %d\n", leftRep);

					}
					else {

						unsigned short topRep = reps[topLabel];
						counter = 0;	
						while(reps[topRep] != topRep){
							topRep = reps[topRep];
							counter++;
							if(counter > 100000){
								printf("Kraut3");
								exit(1);
							}
	
						}



						if(leftRep == topRep){
							out[i*width + j] = leftLabel;
							blobSizes[leftRep]++;
							//printf("topRep = leftRep for %d\n", leftRep);
						}
						else {

							//printf("Cluster Fusion!!!!!!!!!!!!!!!!\n");

							if(leftRep < topRep){
								out[i*width + j] = leftLabel;

								reps[topRep] = leftRep;

								blobSizes[leftRep] += blobSizes[topRep] + 1;
	
							}
							else {

								out[i*width + j] = leftLabel;

								reps[leftRep] = topRep;

								blobSizes[topRep] += blobSizes[leftRep] + 1;

							}

						}

					}

				}
				else if(currColor == top){

					unsigned short topRep = reps[topLabel];
					counter = 0;	
					while(reps[topRep] != topRep){
						topRep = reps[topRep];
						counter++;
						if(counter > 100000 - 5){
							printf("rep: %d reps[reps]: %d Label: %d\n", topRep, reps[topRep], topLabel);

						}
						if(counter > 100000){
							printf("Indices i: %d j: %d\n", i, j);
							printf("NumberOfReps: %d\n", numberOfReps);

							printf("Kraut4");
							exit(1);
						}

					}



					out[i*width + j] = topLabel;
					blobSizes[topRep]++;

					//printf("currColor = top for %d\n", topRep);

				}
				else if(currColor == topLeft){

					unsigned short topLeftRep = reps[topLeftLabel];
					counter = 0;	
					while(reps[topLeftRep] != topLeftRep){
						topLeftRep = reps[topLeftRep];
						counter++;
						if(counter > 100000){
							printf("Kraut5");
							exit(1);
						}

					}



					if(currColor != topRight){
						out[i*width + j] = topLeftLabel;
						blobSizes[topLeftRep]++;
					}
					else {

						unsigned short topRightRep = reps[topRightLabel];
						counter = 0;	
						while(reps[topRightRep] != topRightRep){
							topRightRep = reps[topRightRep];
							counter++;
							if(counter > 100000){
								printf("Kraut6");
								exit(1);
							}
	
						}


						//printf("Cluster Fusion!!!!!!!!!!!!!!!!\n");

						if(topLeftRep < topRightRep){
							out[i*width + j] = topLeftLabel;

							reps[topRightRep] = topLeftRep;

							blobSizes[topLeftRep] += blobSizes[topRightRep] + 1;

						}
						else {

							out[i*width + j] = topLeftLabel;

							reps[topLeftRep] = topRightRep;

							blobSizes[topRightRep] += blobSizes[topLeftRep] + 1;

						}

					}

					//printf("currColor = topLeft for %d\n", topLeftRep);
				}
				else if(currColor == topRight){

					unsigned short topRightRep = reps[topRightLabel];
					counter = 0;	
					while(reps[topRightRep] != topRightRep){
						topRightRep = reps[topRightRep];
						counter++;
						if(counter > 100000){
							printf("Kraut7");
							exit(1);
						}

					}



					out[i*width + j] = topRightLabel;
					blobSizes[topRightRep]++;


				}
				else {
					
					numberOfReps++;
					reps[numberOfReps] = numberOfReps;
					repColors[numberOfReps] = currColor;
					blobSizes[numberOfReps] = 1;
					out[i*width + j] = numberOfReps;
					
				}				
			}

		}
	}

	

/*	for(int i = 1; i < height; i++){
		for(int j = 1; j < width; j++){

			unsigned char currColor = tmp[i*width + j];

			if(valid[currColor] > 0){
				
				unsigned short newRep = reps[out[i*width + j]];
				while(reps[newRep] != newRep)
					newRep = reps[newRep];

				out[i*width + j] = newRep;

			}

		}
	}
*/

/*	printf("\n\nOrg Image \n\n");
	for(int i = 0; i < height; i++){

		for(int j = 0; j < width; j++){
			printf("%d ", tmp[i*width + j]);
		}
		printf("\n");


	}

	printf("\n\nOrg Label \n\n");
	for(int i = 0; i < height; i++){

		for(int j = 0; j < width; j++){
			printf("%d ", out[i*width + j]);
		}
		printf("\n");


	}
*/
	
	
	
	for(unsigned short i = 1; i <= numberOfReps; i++){	
		if(reps[i] == i){
			BlobInformation blob;
			blob.color = repColors[i];
			blob.label = i;
			blob.size = blobSizes[i];
			blobs.push_back(blob);	
			for (unsigned char j =0; j<colors.size();j++) {
				if (blobCount[j].color == blob.color && blob.size > treshold) {
					blobCount[j].blobCount++;
					blobCount[j].pixelCount += blob.size;
					if (blob.size > blobCount[j].maxBlobSize) {
						blobCount[j].maxBlobSize = blob.size;
					}
				}
			}
//			printf("Rep %d for Color %d: %d size %d\n", i, colors[0], reps[i], blobSizes[i]);
		}
		else {
//			printf("Rep %d for Color %d: %d\n", i, colors[0], reps[i]);
		}
	}

	delete[] reps;
	delete[] repColors;
	//delete[] tmp;
	delete[] blobSizes;

}

void FilterHillClimbToSeg::set_pixel(Point q) {
	m_linePix.push_back(q);
}

void FilterHillClimbToSeg::bresenham_linie(Point P, Point q)                  // zeichnet Linie von P nach q        
{                                                      
	m_linePix.clear();
	Point p;
	p.x = P.x;
	p.y = P.y;
	int error, delta, schwelle, dx, dy, inc_x, inc_y;

	dx = q.x - p.x;
	dy = q.y - p.y;

	if (dx>0) inc_x= 1; else inc_x=-1;
	if (dy>0) inc_y= 1; else inc_y=-1;

	if (abs(dy) < abs(dx)) {  // flach nach oben oder flach nach unten

		error = -abs(dx);
		delta = 2*abs(dy);
		schwelle = 2*error;
		while (p.x != q.x) {
			set_pixel(p);
			p.x+=inc_x;
			error = error + delta;
			if (error >0) { p.y+=inc_y; error = error + schwelle;}
		}
	} 

	else                              // steil nach oben oder steil nach unten
       
	{

		error = -abs(dy);
		delta = 2*abs(dx);
		schwelle = 2*error;
		while (p.y != q.y) {
			set_pixel(p);
			p.y+=inc_y;
			error = error + delta;
			if (error >0) { p.x+=inc_x; error = error + schwelle;}
		}
	}         

	set_pixel(q);

}

bool FilterHillClimbToSeg::isLinePix(Point q) {
	for (int i = 0; i<m_linePix.size(); i++) 
	{
		Point p = m_linePix[i];
		if (q.x == p.x && q.y == p.y) {
			return true;
		}
	}
	return false;
}

