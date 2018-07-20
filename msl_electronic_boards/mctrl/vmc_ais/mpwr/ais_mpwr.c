//////////////////////////////////////////////////////////////////////////////
/// \defgroup motor Motor
//@{
/// \file ais_mpwr.c
///
/// \brief motor functions
///
/// \author Jan Paulus, Adam Cwientzek
///
/// \version 0.9
///
/// \date 03.10.2006
///
//////////////////////////////////////////////////////////////////////////////

#include <reg167.h>
#include "aisc167b/ais_pwm.h"
#include "mpwr/ais_mpwr.h"


//////////////////////////////////////////////////////////////////////////////
/// \brief Function to inizial MPWR board
/// \return 1 = error
/// \return 0 = success
//////////////////////////////////////////////////////////////////////////////
void mpwr_init() {
//////////////////////////////////////////////////////////////////////////////
	VMC_UCHAR_8 motor_id;
	// Port for FANcontrol is output
	dir_FanOut = 1;

	// Initialize motor control
	for ( motor_id = 0; motor_id < _NUM_MOTORS_; motor_id++)
		pwm_init(motor_id); // Init PWM Module

	// Init motor 1 output ports
	dp_dir1_pwm0 = 1;
	dp_dir2_pwm0 = 1;

	// Init motor 2 output ports
	dp_dir1_pwm1 = 1;
	dp_dir2_pwm1 = 1;

	// Init motor 3 output ports
	dp_dir1_pwm2 = 1;
	dp_dir2_pwm2 = 1;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Function to set the motor direction
///	\param motor_id Number of the Motor 0 to _NUM_MOTOR_-1
///	\param motor_dir direction of motor
/// \note motor_dir: _MOTORSTATE_FREE_  = free wheel
/// 				 _MOTORSTATE_BREAK_ = break
///					 _MOTORSTATE_FWD_   = forward
///					 _MOTORSTATE_REV_   = reverse
/// \attention _MOTORSTATE_BREAK_ will can cause very high currents !!
/// \return 1 = error
/// \return 0 = success
//////////////////////////////////////////////////////////////////////////////
int mpwr_motor_dir(VMC_UCHAR_8 motor_id, VMC_UCHAR_8 motor_dir) {
//////////////////////////////////////////////////////////////////////////////
 switch ( motor_id ) {
	// Motor 1
 	case 0 :
		switch ( motor_dir ) {
			case _MOTORSTATE_FREE_  : dir1_pwm0 = 0; dir2_pwm0 = 0; break;
			case _MOTORSTATE_BREAK_ : dir1_pwm0 = 1; dir2_pwm0 = 1; break;
			case _MOTORSTATE_FWD_   : dir1_pwm0 = 1; dir2_pwm0 = 0;	break;
			case _MOTORSTATE_REV_ 	: dir1_pwm0 = 0; dir2_pwm0 = 1;	break;
			default: return 1;	// Error, unknown direction
		}
		break;

	// Motor 2
	case 1 :
		switch ( motor_dir ) {
			case _MOTORSTATE_FREE_  : dir1_pwm1 = 0; dir2_pwm1 = 0; break;
			case _MOTORSTATE_BREAK_ : dir1_pwm1 = 1; dir2_pwm1 = 1; break;
			case _MOTORSTATE_FWD_   : dir1_pwm1 = 1; dir2_pwm1 = 0;	break;
			case _MOTORSTATE_REV_ 	: dir1_pwm1 = 0; dir2_pwm1 = 1;	break;
			default: return 1;	// Error, unknown direction
		}
		break;

	// Motor 3
	case 2 :
		switch ( motor_dir ) {
			case _MOTORSTATE_FREE_  : dir1_pwm2 = 0; dir2_pwm2 = 0; break;
			case _MOTORSTATE_BREAK_ : dir1_pwm2 = 1; dir2_pwm2 = 1; break;
			case _MOTORSTATE_FWD_   : dir1_pwm2 = 1; dir2_pwm2 = 0;	break;
			case _MOTORSTATE_REV_ 	: dir1_pwm2 = 0; dir2_pwm2 = 1;	break;
			default: return 1;	// Error, unknown direction
		}
		break;
	default: return 1;	// Error, motor_id out of range
	}

 return 0; // ok
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Function to set the pulse width and direction of one motor
///	\param motor_id Motor 0 to _NUM_MOTORS_ - 1
///	\param signal pulse width -SIGNAL_RANGE to +SIGNAL_RANGE
/// \return 1 = error
/// \return 0 = success
//////////////////////////////////////////////////////////////////////////////
int mpwr_motor_set(VMC_INT_16 motor_id, VMC_INT_16 signal) {
//////////////////////////////////////////////////////////////////////////////
	unsigned int  pw	= 0;
	unsigned char dir	= 0;

	// check if motor_id is within range
	if( !_IS_MOTORID_(motor_id) ) return 1;

	// check signal and generate direction bits
	if(signal == 0){
		dir = 0;
		pw = 0;
	}
	if(signal < 0){
		dir = 3;
		pw = signal * (-1);
	}
	if(signal > 0){
		dir = 2;
		pw = signal;
	}

	// converts the signal into PWM value
	pw = (int)(((long)pw * PWM_PERIOD) / SIGNAL_RANGE);

	if ( pwm_set(motor_id,pw) ) 		return 1;
	if ( mpwr_motor_dir(motor_id,dir) )	return 1;

	return 0;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Initialize H-bridge Error Signal Detection
///
//////////////////////////////////////////////////////////////////////////////
void init_bridge_error() {
    // Attention: Hardware access
    dirP20 = 0;
	dirP21 = 0;
	dirP22 = 0;
	dirP23 = 0;
	dirP24 = 0;
	dirP25 = 0;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the H-Bridge State
/// \param ID Number of the H-Bridge same to the motor number
/// \return 0 everything is OK
/// \return 1 Open load at high-side switch 1
/// \return 2 Open load at high-side switch 2
/// \return 3 Overtemperature
//////////////////////////////////////////////////////////////////////////////
int bridgestate(unsigned char ID) {
	// Attention: Hardware access
	switch(ID) {
		case 0 :
			if(dir1_pwm0 == 0 && ERRA1 == 0 && ERRA2 == 1)
				return 1;

			if(dir2_pwm0 == 0 && ERRA1 == 1 && ERRA2 == 0)
				return 2;

			if(dir1_pwm0 == 1 && dir2_pwm0 == 1 && ERRA1 != ERRA2)
				return 3;

			if(dir1_pwm0 == 1 && dir2_pwm0 == 0 && ERRA1 == 0 && ERRA2 == 1)
				return 3;

			if(dir1_pwm0 == 0 && dir2_pwm0 == 1 && ERRA1 == 1 && ERRA2 == 0)
				return 3;

			return 0;
		break;

		case 1 :
			if(dir1_pwm1 == 0 && ERRB1 == 0 && ERRB2 == 1)
				return 1;

			if(dir2_pwm1 == 0 && ERRB1 == 1 && ERRB2 == 0)
				return 2;

			if(dir1_pwm1 == 1 && dir2_pwm1 == 1 && ERRB1 != ERRB2)
				return 3;

			if(dir1_pwm1 == 1 && dir2_pwm1 == 0 && ERRB1 == 0 && ERRB2 == 1)
				return 3;

			if(dir1_pwm1 == 0 && dir2_pwm1 == 1 && ERRB1 == 1 && ERRB2 == 0)
				return 3;

			return 0;
		break;

		case 2 :
			if(dir1_pwm2 == 0 && ERRC1 == 0 && ERRC2 == 1)
				return 1;

			if(dir2_pwm2 == 0 && ERRC1 == 1 && ERRC2 == 0)
				return 2;

			if(dir1_pwm2 == 1 && dir2_pwm2 == 1 && ERRC1 != ERRC2)
				return 3;

			if(dir1_pwm2 == 1 && dir2_pwm2 == 0 && ERRC1 == 0 && ERRC2 == 1)
				return 3;

			if(dir1_pwm2 == 0 && dir2_pwm2 == 1 && ERRC1 == 1 && ERRC2 == 0)
				return 3;

			return 0;
		break;
	}

	return 0;
}
//////////////////////////////////////////////////////////////////////////////




// --------------------------------------------------------------------
 // Port 1, 2, 3 (not implemented yet)
int mpwr_get_port(VMC_UCHAR_8 portID) {
 VMC_UCHAR_8 temp;
 temp = portID;
 return 0;
}
// --------------------------------------------------------------------


// --------------------------------------------------------------------
 // Channel 1, 2 (not implemented yet)
int mpwr_get_adin(VMC_UCHAR_8 channelID) {
 VMC_UCHAR_8 temp;
 temp = channelID;
 return 0;
}
// --------------------------------------------------------------------


//@}
