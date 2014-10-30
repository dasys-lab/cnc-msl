#include "FilterAdjustGain.h"
#include "../helpers/KeyHelper.h"

FilterAdjustGain::FilterAdjustGain(int width,int height):Filter(OF_ZERO, width, height){

	this->sc = SystemConfig::getInstance();

	init();
}

void FilterAdjustGain::init()
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

FilterAdjustGain::~FilterAdjustGain() {
}
#ifdef OLDLIBDC
int FilterAdjustGain::process(Camera1394 &camera,  FilterDistanceProfileNew &distProfile, std::vector<LinePoint> &linePoints, Particle &particle ) 
#else
int FilterAdjustGain::process(camera::ImagingSource &camera, FilterDistanceProfileNew &distProfile, std::vector<LinePoint> &linePoints, Particle &particle ) 
#endif
{
	if(KeyHelper::checkKey('g')) {
		fixedThreshold--;
		printf("Parameter Adjustment - AutoGainThres: %d\n", (int)fixedThreshold);
	} else if (KeyHelper::checkKey('G')) {
		fixedThreshold++;
		printf("Parameter Adjustment - AutoGainThres: %d\n", (int)fixedThreshold);
	}

	nextUpdate--;
	if(nextUpdate < 0)
		nextUpdate = 0;

	if (frame_count == 0)
	{
#ifdef OLDLIBDC
		camera.enableAutoGain();
#else
		 camera.enableAutoGain(true);
#endif

	}

	else if (frame_count == 30)
	{
		//gain = camera.getGain();
#ifdef OLDLIBDC
		camera.disableAutoGain();
#else		
		camera.enableAutoGain(false);
#endif
		gain = (gain - 200 > 180) ? gain-200 : 180;
		camera.setGain(gain);
		std::cout << "Gain set manuel to: " << gain << std::endl;
		adjustGain = true;
		++frame_count;
		lastThreshold = distProfile.getThreshold();
	}

	if (!adjustGain)
		frame_count++;

		std::cout << "Gain: LP:" << linePoints.size() << " partWeight:"	<< particle.weight << " TH: " << distProfile.getThreshold() << "last gain:" << gain << std::endl;


	if (adjustGain && nextUpdate == 0)
	{
		if (true || (linePoints.size() < 150 && particle.weight < 0.996))
		{
			std::cout << "Adjust Gain: " << std::endl;
			if (distProfile.getThreshold() < fixedThreshold)
			{
				gain = (gain + 10 < 1000)? gain + 10 : 1000;
				if(gain > 1000)
					gain = 1000;
				camera.setGain(gain);
				std::cout << "Gain increased: " << gain << std::endl;
			}
			else
			{
				gain = (gain - 10 > 1) ? gain-10 : 1;
				if(gain < 300)
					gain = 300;
				camera.setGain(gain);
				std::cout << "Gain decreased: " << gain << std::endl;
			}
		}
		nextUpdate = 3;
	}

	return gain;
}

/*
int FilterAdjustGain::process(Camera1394 &camera, FilterDistanceProfileNew &distProfile, 			std::vector<LinePoint> &linePoints, Particle &particle ) {

	frame_count++;

	if (frame_count == 1)
	{
		camera.disableAutoGain();
		adjustGain = false;
		gain = 1000;
		count = 0;
	}


	if (adjustGain)
	{
		std::cout << "Gain: LP:" << linePoints.size() << " partWeight:"	<< particle.weight << " TH: " << distProfile.getThreshold() << "last gain:" << gain << std::endl;
		if (linePoints.size() < 100 || particle.weight < 0.98)
		{
			if (initStep)
			{
				gain = gain-10 < 300 ? 1023: gain-10;
				std::cout << "Gain: INIT set to: " << gain << std::endl;
				camera.setGain(gain);
			}
			else
			{
				std::cout << "Gain normal adjusting " << std::endl;
				if (distProfile.getThreshold() > lastThreshold)
				{
					gain = gain -10 > 1 ? gain-10 : 1;
				}
				else
				{
					gain = gain +10 < 1000 ? gain+10 : 1000;
				}
				std::cout << "Gain: adjusting set to: " << gain << std::endl;
				camera.setGain(gain);

			}

			adjustGain = false;
			count = 0;
		}
		else
		{
			if (initStep)
			{
				initStep = false;
				lastThreshold = distProfile.getThreshold();
				adjustGain = false;
				count = 0;
				std::cout << "gain init false gesetzt" << std::endl;
			}


		}
	}
	else // !adjustGain
	{
		std::cout << "Gain not adjust" << std::endl;
		if (++count == 5)
		{
			adjustGain = true;
		}
	}
}*/
