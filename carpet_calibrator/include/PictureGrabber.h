/*
 * PictureGraber.h
 *
 *  Created on: 06.05.2015
 *      Author: tobi
 */

#ifndef CNC_MSLDRIVER_CARPET_CALIBRATOR_SRC_PICTUREGRABBER_H_
#define CNC_MSLDRIVER_CARPET_CALIBRATOR_SRC_PICTUREGRABBER_H_

#include <SystemConfig.h>

namespace msl_vision
{

	class PictureGrabber
	{
	public:
		PictureGrabber();
		virtual ~PictureGrabber();

	private:
		supplementary::SystemConfig* sc;
	};

}

#endif /* CNC_MSLDRIVER_CARPET_CALIBRATOR_SRC_PICTUREGRABER_H_ */
