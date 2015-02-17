//////////////////////////////////////////////////////////////////////////////
/// \ingroup motor Motor
//@{
/// \file ais_mpwr.h
///
/// \brief Header file for the motor functions
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

#ifndef _AIS_MPWR_H_
#define _AIS_MPWR_H_

#include <reg167.h>

#define	_MOTORSTATE_FREE_   0
#define	_MOTORSTATE_BREAK_  1
#define	_MOTORSTATE_FWD_	2
#define	_MOTORSTATE_REV_	3


sbit dir1_pwm0   	= P7^4;		// Pin 7.4: PWM output channel left
sbit dp_dir1_pwm0  	= DP7^4;    // Direction register 7.4
sbit dir2_pwm0   	= P7^5;     // Pin 7.5: PWM output channel left
sbit dp_dir2_pwm0  	= DP7^5;    // Direction register 7.5

sbit dir1_pwm1   	= P7^6;     // Pin 7.6: PWM output channel left
sbit dp_dir1_pwm1  	= DP7^6;    // Direction register 7.6
sbit dir2_pwm1   	= P7^7;     // Pin 7.7: PWM output channel left
sbit dp_dir2_pwm1  	= DP7^7;    // Direction register 7.7

sbit dir1_pwm2   	= P8^1;     // Pin 8.1: PWM output channel left
sbit dp_dir1_pwm2  	= DP8^1;    // Direction register 8.1
sbit dir2_pwm2   	= P8^3;     // Pin 8.3: PWM output channel left
sbit dp_dir2_pwm2  	= DP8^3;    // Direction register 8.3


// Direction register for Port 2.0 - 2.5
sbit dirP20      = DP2^0;
sbit dirP21      = DP2^1;
sbit dirP22      = DP2^2;
sbit dirP23      = DP2^3;
sbit dirP24      = DP2^4;
sbit dirP25      = DP2^5;
sbit dir_FanOut  = DP2^15;
sbit io_FanOut   = P2^15;

//sbit dirIOAin
//sbit dirIOBin
//sbit dirIOCin


// H-Bridge Error Signals on Port 2.0 - 2.5
sbit ERRA1    = P2^0;
sbit ERRA2    = P2^1;
sbit ERRB1    = P2^2;
sbit ERRB2    = P2^3;
sbit ERRC1    = P2^4;
sbit ERRC2    = P2^5;


void mpwr_init();
int mpwr_motor_dir(VMC_UCHAR_8, VMC_UCHAR_8);
int mpwr_motor_set(VMC_INT_16 motor_id, VMC_INT_16 signal);
int mpwr_get_port(VMC_UCHAR_8 portID); // Port 1, 2, 3
int mpwr_get_adin(VMC_UCHAR_8 channelID); // Channel 1, 2



void init_bridge_error();
int bridgestate(VMC_UCHAR_8 ID);

#endif /* _AIS_MPWR_H_ */

//@}
