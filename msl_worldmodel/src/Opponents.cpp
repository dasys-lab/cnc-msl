/*
 * Opponents.cpp
 *
 *  Created on: Feb 26, 2016
 *      Author: Stefan Jakob
 */

#include "Opponents.h"

namespace msl
{

	Opponents::Opponents(MSLWorldModel* wm, int ringBufferLength)
	{
		this->wm = wm;
		this->ringBufferLength = ringBufferLength;
	}

	Opponents::~Opponents()
	{
		// TODO Auto-generated destructor stub
	}

} /* namespace msl */
