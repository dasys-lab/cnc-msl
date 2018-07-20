//////////////////////////////////////////////////////////////////////////////
/// \defgroup adc AD-Converter
//@{
/// \file ais_adc.c
///
/// \brief Analog-Digital conversion functions
///
/// \author Jan Paulus
///
/// \version 0.8
///
/// \date 18.07.2005
///
//////////////////////////////////////////////////////////////////////////////

#include <reg167.h>
#include "aisc167b/ais_adc.h"

//////////////////////////////////////////////////////////////////////////////
/// \brief AD conversion of all analogue input channels (0 - 15).
/// 			 For each channel a 10-bit digital value is delivered, which is transmitted in 2 bytes.
/// \param adc_data pointer of a unsigned int arrey with the lenght of 15
/// \note the function will fill up the arrey with the AD datas from the channels
/// \attention if the arrey is to sort there will be a invalid memory access
//////////////////////////////////////////////////////////////////////////////
void get_all_adc(VMC_UINT_16 *adc_data) {
    VMC_INT_16 i;
    for (i = 0; i < 15; i++) {
        adc_data[i] = get_sc_adc(i);
	}
    return;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief AD conversion of one analogue input channel. (Fixed Channel Single Conversion)
///        For the channel a 10-bit digital value is delivered, which is transmitted in 2 bytes.
/// \param x Channel number
/// \returns data of the AD-conversion
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_sc_adc (VMC_INT_16 x) {
    if(x < 0 || x > 15) {			// is the channel out of range
	    return 0;
    }
    ADCON = x;								// Select channel to convert
    ADST  = 1;								// Begin conversion
    while (ADBSY);						// Wait while the ADC is converting
    return(ADDAT) & 0x03FF;	// Return the result
}

//////////////////////////////////////////////////////////////////////////////
/// \brief gets the current signal from the AD-Converter and calculate the actual curren in [mA]
/// \param Motor_ID Motor number
/// \returns actual current of one Motor in [mA]
///	\returns -1 if the Motor_id is invalid
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_current(VMC_UCHAR_8 Motor_ID) {

	switch (Motor_ID) {
		case 0 :
			return (((((float)get_sc_adc(9))*5)/1023)/0.56) *1000;
		break;
		case 1 :
			return (((((float)get_sc_adc(8))*5)/1023)/0.56) *1000;
		break;
		case 2 :
			return (((((float)get_sc_adc(5))*5)/1023)/0.56) *1000;
		break;
	}

	return -1;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief gets the voltage signal from the AD-Converter and calculate the actual battery voltage in [V]
/// \returns actual battery voltage in [V]
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_batteryvoltage(void) {
    // Attention: Hardware access
//    return (int)((((long)get_sc_adc(0))*5*10000)/1023);
    return (int)((((long)get_sc_adc(0))*55314)/1000);
}

//@}
