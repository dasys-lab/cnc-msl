/*
 * BayerDemosaic.cpp
 *
 *  Created on: Jun 8, 2017
 *      Author:  Lisa Martmann
 */

#include "msl_ptgrey_camera/filters/BayerDemosaic.h"
namespace msl_ptgrey_camera
{

	BayerDemosaic::BayerDemosaic()
	{

	}

	BayerDemosaic::~BayerDemosaic()
	{
		// TODO Auto-generated destructor stub
	}

	void BayerDemosaic::demosaic(CNImage* origin)
	{

		const int kernelRB[3][3] = { {1, 2, 1}, {2, 4, 2}, {1, 2, 1}};

		const int kernelG[3][3] = { {0, 1, 0}, {1, 4, 1}, {0, 1, 0}};




	}

} // namespace msl_ptgrey_camera

