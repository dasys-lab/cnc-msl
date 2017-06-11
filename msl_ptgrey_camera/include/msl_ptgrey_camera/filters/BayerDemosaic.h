/*
 * BayerDemosaic.cpp
 *
 *  Created on: Jun 8, 2017
 *      Author:  Lisa Martmann
 */

#ifndef INCLUDE_MSL_PTGREY_CAMERA_FILTERS_BAYERDEMOSAIC_H_
#define INCLUDE_MSL_PTGREY_CAMERA_FILTERS_BAYERDEMOSAIC_H_

#include "msl_ptgrey_camera/CNImage.h"
#include <math.h>
#include <cstdint>
namespace msl_ptgrey_camera
{
	class BayerDemosaic
	{
	public:
		BayerDemosaic();
		virtual ~BayerDemosaic();
		void demosaic(CNImage* origin);
	protected:

	private:
	};

} /* namespace msl_ptgrey_camera */

#endif /* INCLUDE_MSL_PTGREY_CAMERA_FILTERS_BAYERDEMOSAIC_H_ */
