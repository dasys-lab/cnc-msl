#pragma once
#include "RosMsgReceiver.h"
#include <math.h>
#include <fstream>
#include <list>
#include <global/CoxTypes.h>


class MapHelper {
	public:	
		static MapHelper* getInstance();
		void initializeMap();
		unsigned char* getMap() {return newMap;}
		
		double WIDTH;
		double HEIGHT;
		double IWIDTH;
		double IHEIGHT;
		double RESOLUTION;
		
		
		int inline id(int x, int y, int width) {return x+y*width;}
		int inline idx(int id, int width) {return id/width;}
		int inline idy(int id, int width) {return id%width;}
		
		int minXLocation, maxXLocation, minYLocation, maxYLocation;
		
		double fxGradient(int indX, int indY);
		double fyGradient(int indX, int indY);
		double fangleGradient(int px, int py, double pangle, double lx, double ly);
		float* getDistanceMap() {return wallDistanceMap;};
		float getDistance(int X, int Y) {return wallDistanceMap[id(X, Y, IWIDTH)];};
		
		bool gradientAvailable() {return derivation;};
		Cox::field_circles_t getCircles();
		Cox::field_lines_t getLines();
	
	private:
		bool derivation;
		MapHelper();
		void initializeGradientMap();
		static MapHelper* instance;
		unsigned char* distMap;
		unsigned char* newMap;
		
		float* wallDistanceMap;
		float* fdistMap;
		float* fXSobel;
		float* fYSobel;
		
		signed char* xSobelMap;
		signed char* ySobelMap;
		float __max_x, __max_y, __min_x, __min_y;
};
