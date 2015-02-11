//////////////////////////////////////////////////////////////////////////////
/// \ingroup pwm PWM
//@{
/// \file ais_pwm.h
///
/// \brief Header File for the PWM functions
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

#ifndef _AIS_PWM_H_
#define _AIS_PWM_H_

#include <reg167.h>
#include "ais_typedef.h"


/// Resolution of pulse width modulation.
// 255   =  8Bit PWM resolution; frequency 78.13 kHz
// 1023  = 10Bit PWM resolution; frequency 19.53 kHz
// 4095  = 12Bit PWM resolution; frequency  4.88 kHz
// 16383 = 14Bit PWM resolution; frequency  1.22 kHz
// 65535 = 16Bit PWM resolution; frequency    305 Hz (maximum resolution otherwise unsigned int overflow)
#define PWM_PERIOD 1023


sbit dp_pwm0  = DP7^0;       /**< Direction register 7.0 for PWM0*/
sbit dp_pwm1  = DP7^1;       /**< Direction register 7.1 for PWM1*/
sbit dp_pwm2  = DP7^2;       /**< Direction register 7.1 for PWM2*/
sbit dp_pwm3  = DP7^3;       /**< Direction register 7.1 for PWM3*/

int pwm_init(VMC_UCHAR_8);
int pwm_set(VMC_UCHAR_8, VMC_UINT_16);

#endif /* _AIS_PWM_H_ */

//@}
