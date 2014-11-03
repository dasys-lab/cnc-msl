#include "FilterAdjustGainNew.h"

FilterAdjustGainNew::FilterAdjustGainNew(int width,int height):Filter(OF_ZERO, width, height){

	this->sc = SystemConfig::getInstance();

	init();
}

void FilterAdjustGainNew::init()
{
	frame_count = 0;
	adjustGain = false;
	initStep = true;
	nextUpdate = 30;
	gain = 500;

	Configuration *vision = (*this->sc)["Vision"];

	fixedThreshold = vision->get<short>("Vision", "Camera1394Settings", "AutoGainThres", NULL);

	//fixedThreshold = 12.0;

	//char * envThreshold = getenv("VISION_GAIN_THRES");
	//if(envThreshold != NULL)
	//	fixedThreshold = (double) atoi(envThreshold);


}

FilterAdjustGainNew::~FilterAdjustGainNew() {
}
#ifdef OLDLIBDC
//camera::ImagingSource
int FilterAdjustGainNew::process(Camera1394 &camera, unsigned char* src, int area, FilterDistanceProfileNew &distProfile, std::vector<LinePoint> &linePoints, Particle &particle ) 
#else
int FilterAdjustGainNew::process(camera::ImagingSource &camera, unsigned char* src, int area, FilterDistanceProfileNew &distProfile, std::vector<LinePoint> &linePoints, Particle &particle )
#endif
{
		int gain = camera.getGain();
		long intensity = 0;
#ifdef OLDLIBDC
		camera.disableAutoGain();
#else		
		camera.enableAutoGain(false);
#endif

		for(int x=0; x<10; x++) {
			for(int y=0; y<10; y++) {
				intensity+=src[x+area*y];
				intensity+=src[(area-1)-x+area*y];
				intensity+=src[x+area*((area-1)-y)];
				intensity+=src[(area-1)+area*((area-1)-y)];
			}
		}

		std::cout << "Gain: LP:" << linePoints.size() << " partWeight:" << particle.weight << " TH: " << distProfile.getThreshold() << "last gain:" << gain << " Cornerintensity: " << intensity << std::endl;

		if(intensity < fixedThreshold-10) {
			gain+=5;
			camera.setGain(gain);
		}
		else if(intensity > fixedThreshold+10) {
			gain-=5;
			camera.setGain(gain);
		}
		return 1;
}


