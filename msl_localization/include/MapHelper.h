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
		
		
		int inline id(int x, int y, int width) {return x*width+y;}
		int inline idx(int id, int width) {return id%width;}
		int inline idy(int id, int width) {return id/width;}
		
		int minXLocation, maxXLocation, minYLocation, maxYLocation;
		
		double angleGradient(double px, double py, double pangle, double lx, double ly);
		double angleGradient(int px, int py, double pangle, double lx, double ly);
		double xGradient(double x, double y);
		double xGradient(int indX, int indY);
		double yGradient(double x, double y);
		double yGradient(int indX, int indY);
		
		double fxGradient(int indX, int indY);
		double fyGradient(int indX, int indY);
		double fangleGradient(int px, int py, double pangle, double lx, double ly);
		double fangleGradient(double px, double py, double pangle, double lx, double ly);
		float* getDistanceMap() {return wallDistanceMap;};
		float getDistance(int X, int Y) {return wallDistanceMap[id(Y, X, IWIDTH)];};
		
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
};
