/*
 * lightbarrier.h
 *
 *  Created on: Mar 7, 2016
 *      Author: Lukas Will
 */

#ifndef INCLUDE_LIGHTBARRIER_H_
#define INCLUDE_LIGHTBARRIER_H_

#include "BlackDef.h"
#include "BlackADC.h"


class LightBarrier {
	private:
		BlackLib::BlackADC	*adc;

		int		threshold;


	public:
				LightBarrier(BlackLib::adcName adc_P);
				~LightBarrier();

		bool	checkLightBarrier();
		bool	setTreshold(int th);

};



#endif /* INCLUDE_LIGHTBARRIER_H_ */
