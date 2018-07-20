//////////////////////////////////////////////////////////////////////////////
/// \defgroup pwm PWM
//@{
/// \file ais_pwm.c
///
/// \brief PWM functions
///
/// \author Jan Paulus
///
/// \version 0.8
///
/// \date 18.07.2005
///
//////////////////////////////////////////////////////////////////////////////

#include <reg167.h>
#include "aisc167b/ais_pwm.h"

//////////////////////////////////////////////////////////////////////////////
/// \brief Function to initial the PWM Signals
/// \param channel PWM Channel to initial
///	\note the resolution of pulse width modulation can be changed by PWM_PERIOD in the header-file
/// \return 1 = error
/// \return 0 = success
//////////////////////////////////////////////////////////////////////////////
int pwm_init(VMC_UCHAR_8 channel) {
	// Attention: Hardware access
	if(channel < 0 || channel > 3)
		return 1;

	switch ( channel ) {
		case 0 :
			dp_pwm0 = 1;
			PP0  = PWM_PERIOD;      // Set PWM period
	 		PW0  = PWM_PERIOD +1;		// Set 0% duty cycle
			PB01 = 0;								// Channel 0 & 1 are independent
			PWMCON0 = PWMCON0 | 0x01;  // Start the PTx counter
			PWMCON1 = PWMCON1 | 0x01;  // Enable channel output
		break;

		case 1 :
			dp_pwm1 = 1;
			PP1  = PWM_PERIOD;			// Set PWM period
			PW1  = PWM_PERIOD +1;		// Set 0% duty cycle
			PB01 = 0;								// Channel 0 & 1 are independent
			PWMCON0 = PWMCON0 | 0x02;  // Start the PTx counter
			PWMCON1 = PWMCON1 | 0x02;  // Enable channel output
		break;

		case 2 :
			dp_pwm2 = 1;
			PP2  = PWM_PERIOD;			// Set PWM period
			PW2  = PWM_PERIOD +1;		// Set 0% duty cycle
			PS2 = 0;								// Standard mode (non-single shot)
			PWMCON0 = PWMCON0 | 0x04;  // Start the PTx counter
			PWMCON1 = PWMCON1 | 0x04;  // Enable channel output
		break;

		case 3 :
			dp_pwm3 = 1;
			PP3  = PWM_PERIOD;			// Set PWM period
			PW3  = PWM_PERIOD +1;		// Set 0% duty cycle
			PS3 = 0;								// Standard mode (non-single shot)
			PWMCON0 = PWMCON0 | 0x08;  // Start the PTx counter
			PWMCON1 = PWMCON1 | 0x08;  // Enable channel output
		break;
	}

	return 0;
}


//////////////////////////////////////////////////////////////////////////////
/// \brief Function to set the pulse width of one PWM Channel
/// \param channel PWM Channel(0-3)
/// \param pw pulse width from 0 (no voltage) to PWM_PERIOD (full power)
/// \return 1 = error
/// \return 0 = success
//////////////////////////////////////////////////////////////////////////////
int pwm_set(VMC_UCHAR_8 channel, VMC_UINT_16 pw) {
	// Attention: Hardware access
	if(channel < 0 || channel > 3)
		return 1;

	if(pw < 0)
		pw = 0;

	if(pw > PWM_PERIOD)
		pw = PWM_PERIOD;

	switch ( channel ) {
		case 0 :
	 		PW0  = ((pw-PWM_PERIOD)*(-1));
		break;

		case 1 :
			PW1  = ((pw-PWM_PERIOD)*(-1));
		break;

		case 2 :
			PW2  = ((pw-PWM_PERIOD)*(-1));
		break;

		case 3 :
			PW3  = ((pw-PWM_PERIOD)*(-1));
		break;
 	}
 return 0;
}

//@}
