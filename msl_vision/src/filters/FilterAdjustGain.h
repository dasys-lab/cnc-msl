#ifndef ADJUSTGAIN_H
#define ADJUSTGAIN_H

#include "Filter.h"

#include "FilterDistanceProfileNew.h"

#ifdef OLDLIBDC
#include "../Camera1394.h"    //new Cam Driver
#else
#include "../driver/imagingsource.h" //BUG have to be fixed
#endif

#include "../helpers/ParticleFilter.h"
#include "../helpers/LinePoint.h"

#include <SystemConfig.h>

class FilterAdjustGain  : public Filter {

	public:
		FilterAdjustGain(int width, int height);
		~FilterAdjustGain();
#ifdef OLDLIBDC
		int process(Camera1394 &camera, FilterDistanceProfileNew &distProfile, std::vector<LinePoint> &linePoints, Particle &particle );
#else		
		int process(camera::ImagingSource &camera, FilterDistanceProfileNew &distProfile, std::vector<LinePoint> &linePoints, Particle &particle );
#endif

	protected:
		SystemConfig* sc;

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
