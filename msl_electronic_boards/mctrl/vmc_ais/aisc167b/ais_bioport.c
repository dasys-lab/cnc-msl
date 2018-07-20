// E:\__Profibot1\AktuelleFirmware\work_95a
#include <reg167.h>

#include "can_stuff/ais_can.h"
#include "system/ais_utils.h"
#include "system/ais_cmdbuff.h"
#include "motorctrl/ais_motorcmd.h"


#include "aisc167b/ais_bioport.h"
#include "aisc167b/ais_adc.h"
//////////////////////////////////////////////////////////////////////////////
/// \brief Initialisation of digtal IO
/// \param motor_id number of motor for which you need the encoder for
//////////////////////////////////////////////////////////////////////////////
void can_debug(unsigned char mark, char index) {
cansend_data[0]= (P3 >> 8) & 0x23;
cansend_data[1]= (DP3>> 8) & 0x23;
cansend_data[2]= (ODP3>>8) & 0x23;
cansend_data[3]= cansend_data[4];
cansend_data[4]= cansend_data[5];
cansend_data[5]= cansend_data[6];
cansend_data[6]= mark;
cansend_data[7]= index;

 can_send(index);

}

void init_bioport(void){
/*
	DIO1IN = 0;			// set all IO pins to 0 (safty first)
	DIO2IN = 0;
	DIO3IN = 0;

	dp_DIO1IN = 0;		// set all IO pins to Input
	dp_DIO2IN = 0;
	dp_DIO3IN = 0;

	odp_DIO1IN = 0;		// pull up all Inputs
	odp_DIO2IN = 0;
	odp_DIO3IN = 0;
 */
#define MASK 0x2300

	P3 = P3 & ~MASK;
	DP3 = DP3  &   ~MASK;
 	ODP3 = ODP3  &   ~MASK;

 	
}

VMC_UCHAR_8 configurate_ioport(VMC_UINT_16 pins) {
VMC_UINT_16 help;

	help = DP3 & ~MASK;
	help |= (pins & 0x03)<<8;
	help |= (pins & 0x04)<<11;

	DP3  = help;


 	//Analog is still to do <<<< there is nothing to do
	return 1;
}

VMC_UINT_16 get_configurate_ioport() {

	VMC_UINT_16 pins;

	pins = (DP3 & 0x0300) >> 8;
	pins = pins | ((DP3 & 0x2000) >> 11);
//	pins = pins | ((P5 & 0x0004) << 1);
//	pins = pins | ((P5 & 0x0010));

	//Analog is still to do  <<< nothing to do - its always zero
	return pins;
}

VMC_UCHAR_8 set_dioport(VMC_UINT_16 pins) {
VMC_UINT_16 help;



	help = (VMC_UINT_16)(pins & 0x03);
	help |= (VMC_UINT_16)(pins & 0x04)<<3;
	help = help << 8;


	help |= (P3 & ~MASK);
	P3    = help;

	//Analog still deals only with input
	return 1;
}

VMC_UINT_16 get_dioport() {
	VMC_UINT_16 pins;
/*
	VMC_UCHAR_8 DIO2IN_ = DIO2IN;
	VMC_UCHAR_8 DIO3IN_ = DIO3IN;
	VMC_UCHAR_8 AIO1IN_ = AIO1IN;
	VMC_UCHAR_8 AIO2IN_ = AIO2IN;

	pins = pins & DIO1IN;
	pins = pins & DIO2IN_ << 1;
	pins = pins & DIO3IN_ << 2;
	pins = pins & AIO1IN_ << 3;
	pins = pins & AIO2IN_ << 4;
*/
	pins = (P3 & 0x0300) >> 8;
	pins = pins | ((P3 & 0x2000) >> 11);
	pins = pins | ((P5 & 0x0004) << 1);
	pins = pins | ((P5 & 0x0010));

	return pins;


//	return value;

}

VMC_UINT_16 get_a1_ioport() {    // dummy for analog port 1

	VMC_UINT_16 value = get_sc_adc(4);

	return value;
}


VMC_UINT_16 get_a2_ioport() {    // dummy for analog port 2

	VMC_UINT_16 value  = get_sc_adc(2);

	return value;
}

enum states bumperstate = BUMPERFREE;
// status flag 
//VMC_UCHAR_8 mode_flag	= 0;	// channel M3 is reserved for two bumpers
VMC_UCHAR_8 mode_flag	= 1;	// free use of M3

VMC_UINT_16 get_bumper_port() {
/**************************************

	bumper ONLY at M3

	get_dioport != 0xc0 >>> all/one bumper pressed

***************************************/

VMC_UCHAR_8 port;

	if (mode_flag)	return 0; 			 // free M3 use
	else {
		port = (VMC_UCHAR_8)(get_dioport() & 0x0c);

		if (port == 0x0c) {
			bumperstate = BUMPERFREE;
			return(0);	 /// no contact
			}
		else 
			if (bumperstate == BUMPERFREE) {
	 // bumper tilt so alter state
				bumperstate = BUMPERTILT;


	// Response to Host 
				{
struct CmdStruct ResponseCmd;
				ERR_range(&ResponseCmd, _CMDGRP_MOTOR_ERR_ );

				ResponseCmd.cmd    = 0x70;    // Same as Request Command
	// Values possible with two bumpers =x00, 0x01, 0x02, 0x03
				ResponseCmd.data[0]    = port >> 2; //(VMC_UCHAR_8)port; 
//				ResponseCmd.data[1]    = port >> 2; //(VMC_UCHAR_8)port; 
//				ResponseCmd.data[2]    = port >> 2; //(VMC_UCHAR_8)port; 
				ResponseCmd.datalen = 1;

	// Send command to command buffer
				cmdb_set_command( &ResponseCmd );
				}
				return (port);
				} //  if bumperstate
	 	return 0;  
	 	}
}
