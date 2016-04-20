/*
 * LightBarrier.cpp
 *
 *  Created on: Apr 18, 2016
 *      Author: Stefan Jakob
 */

#include <LightBarrier.h>
#include "MSLWorldModel.h"
namespace msl
{

	LightBarrier::LightBarrier(MSLWorldModel* wm)
	{
		this->wm = wm;
		this->sc = supplementary::SystemConfig::getInstance();
		this->useLightBarrier = (*this->sc)["LightBarrier"]->get<bool>("LightBarrier", "UseLightBarrier", NULL);
	}

	LightBarrier::~LightBarrier()
	{
	}

	bool LightBarrier::getLightBarrier(int index)
	{
		return this->wm->rawSensorData.getLightBarrier(index);
	}

	bool LightBarrier::mayUseLightBarrier()
	{
		return this->useLightBarrier;
	}

} /* namespace msl */
