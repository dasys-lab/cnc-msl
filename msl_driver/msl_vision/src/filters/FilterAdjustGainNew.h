#ifndef ADJUSTGAINNEW_H
#define ADJUSTGAINNEW_H

#include "Filter.h"

#include "FilterDistanceProfileNew.h"

#ifdef OLDLIBDC
#include "../Camera1394.h"
#else
#include "../driver/imagingsource.h"
#endif

#include "../helpers/ParticleFilter.h"
#include "../helpers/LinePoint.h"

#include <SystemConfig.h>

class FilterAdjustGainNew  : public Filter {

	public:
		FilterAdjustGainNew(int width, int height);
		~FilterAdjustGainNew();

#ifdef OLDLIBDC		
int process(Camera1394 &camera, unsigned char* src, int area, FilterDistanceProfileNew &distProfile, std::vector<LinePoint> &linePoints, Particle &particle);
#else
		int process(camera::ImagingSource &camera, unsigned char* src, int area, FilterDistanceProfileNew &distProfile, std::vector<LinePoint> &linePoints, Particle &particle);
#endif

	protected:
		SystemConfigPtr sc;

		int gain;

		int frame_count;
		int nextUpdate;
		bool adjustGain;
		bool initStep;
		int count;
		double lastThreshold;
		double fixedThreshold;
		void init();


};

#endif
