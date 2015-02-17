//////////////////////////////////////////////////////////////////////////////
/// \defgroup error Check for Errors
//@{
/// \file ais_error.c
///
/// \brief Functions to check for the different Errors, which can occur at the motor or other hardware
///
/// This module contains functions to check if the corresponding Error has occured and set an error fla if applicable\n
/// To use the module you have to initialise it with the function init_Error()\n
/// Then you should call the chk_... funtions periodic, which return you a 1, if an error has occurd or a 0 if not\n
///
/// \note To add a error check funtion to this module, you have to do the following steps:\n
/// 1. Define and comment a function chk_[error] in the ais_error.h\n
/// 2. Implement the function in this file, return char 1 if error 0 if not\n
/// 3. Add a call of this function to the error check chapter of the control_loop() in the module ais_control\n
/// 4. Add the char to the motorstate.error_check struct (ERROR_CHECK) in the ais_control.h\n
///
/// \author Pascal Langenberg
///
/// \version 0.4
///
/// \date 09.08.2006
///
//////////////////////////////////////////////////////////////////////////////


#include "ais_error.h"
#include "control/ais_motorctrl.h"
#include "ais_configdata.h"


// Timeout counter
TMC_UINT_16 timeout;

//////////////////////////////////////////////////////////////////////////////
/// \brief initialization of Error Check variables
//////////////////////////////////////////////////////////////////////////////
void init_Error() {
	timeout = 0;
}
//////////////////////////////////////////////////////////////////////////////

/// \brief Function to check if one motor is available
/// The bridges on the TMC 2000 offer a possibility to check, if there is a motort connected to the bridge or not
/// This function check this and returns an error if the corrsponding bridge signalise an error
/// \param id ID of the motor to check
/// \not not implemented yet!
/// \returns a flag if a there is a problem with the motor 0: ok 1: not ok
//////////////////////////////////////////////////////////////////////////////
TMC_UCHAR_8 chk_Motor(TMC_UCHAR_8 id) {
    return 0;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Function to check if there is to much time since last command
/// \param motor_ID The ID of the motor to check
/// \returns a flag if there is a timeout 0: no timeout 1:timeout
//////////////////////////////////////////////////////////////////////////////
TMC_UCHAR_8 chk_Timeout(TMC_LONG_32 last_cycle_length) {

    if (get_timeout() == 0) return 0;
	// increase timeout counter, which is reseted by any motorcontrol command in ais_motorcmd
	timeout = timeout + (last_cycle_length / 1000);

	// check if timeout
    if (timeout  > get_timeout()) {
		// avoid overflow
		timeout = get_timeout() + 1;
		//return error --> timeout!
	return 1;
	}

	//if no timeout retun no error
	return 0;

}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief resets the timeout counter
//////////////////////////////////////////////////////////////////////////////
void reset_Timeout() {
	timeout = 0;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Function to check if there is a Problem with the encoder signals
///
/// \param motor_ID The ID of the motor to check
/// \returns a flag if there is a problem with the encoder signal
//////////////////////////////////////////////////////////////////////////////
TMC_UCHAR_8 chk_Encoder(TMC_UCHAR_8 id) {
    return 0;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Function to check if an emergency stop is necassary \n
/// If you want to add a condition for an emergencystop, just add an if case which returns a 1 befor the return 0 command
/// \param motor_ID The ID of the motor to check
//////////////////////////////////////////////////////////////////////////////
TMC_UCHAR_8 chk_Emergencystop(struct ERR_STATES *error_sys) {
    return error_sys->timeout;

}
//////////////////////////////////////////////////////////////////////////////

//@}


