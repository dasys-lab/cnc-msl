/// \ingroup adc AD-Converter
//@{
//////////////////////////////////////////////////////////////////////////////
/// \file ais_adc.h
///
/// \brief Header File for AD conversion functions
///
///
///
/// \author Jan Paulus
///
/// \version 0.8
///
/// \date 18.07.2005
///
//////////////////////////////////////////////////////////////////////////////

#ifndef _AIS_ADC_H_
#define _AIS_ADC_H_

#include "ais_typedef.h"

void get_all_adc(VMC_UINT_16 *adc_data);					// conversion of all channels
VMC_UINT_16 get_sc_adc (int x);									// single channel conversion
VMC_UINT_16 get_current(VMC_UCHAR_8 Motor_ID);	// returns the actual current of one motor
VMC_UINT_16 get_batteryvoltage(void);

#endif /* _AIS_ADC_H_ */

//@}
