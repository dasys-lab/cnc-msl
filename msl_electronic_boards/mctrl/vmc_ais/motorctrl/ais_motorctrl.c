//////////////////////////////////////////////////////////////////////////////
/// \defgroup control The Control Loop
//@{
/// \file ais_motorctrl.c
///
/// \brief Control Loop and runtime data
///
/// This module contains the control loop function and the runtime data of the TMC 2000 firmware \n
/// To use this module you first have to initialise it with the function init_Controldata() \n
/// The funtion control_loop() has then to be called periodic with the last cycle length in microseconds as unsigned long as parameter\n
/// For the runtime data there exists set and get functions to build status responses(get) and control the motor(set)\n
///
/// \note If you want to add a new runtime data to the module you have to do the following steps:\n
/// 1. Add a variable declaration and a comment to the struct MOTORSTATE in the ais_control.h file\n
/// 2. Add a calculation function for this variable to the modul ais_calculatestate, see there how to do\n
/// 3. Add a call of this fucntion into the calculating chapter of the function control_loop, use existing as template\n
/// 4. Add the variable to the module ais_motorcmd, see there how to do\n
///
///
/// \author Pascal Langenberg
///
/// \version 0.4
///
/// \date 09.08.2006
///
///
///
//////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include "ais_typedef.h"
#include "aisc167b/ais_pwm.h"			// needed by convert RPM2PWM
#include "mpwr/ais_mpwr.h"
#include "motorctrl/ais_motorctrl.h"
#include "motorctrl/ais_motorconfig.h"
#include "motorctrl/ais_error.h"
#include "motorctrl/ais_calculatestate.h"
#include "motorctrl/ais_limiter.h"

#include "controller/ais_controller.h"
#include "motorctrl/ais_output.h"


// declaration of one motormotorstate structure per motor
struct MOTORSTATE motorstate[_NUM_MOTORS_];


// declaration of one structure representing the mode of the controller
struct ERR_STATES error_sys;


//////////////////////////////////////////////////////////////////////////////
/// \brief initialization of Control Loop Variables
//////////////////////////////////////////////////////////////////////////////
void init_motorcontrol() {

    //loop counter
	VMC_UCHAR_8 motorID;

	error_sys.timeout		= 0; // No Timeout
	error_sys.emergencystop  = 0; // No Error

	for(motorID = 0; motorID < _NUM_MOTORS_; motorID++) {
		motorstate[motorID].ID		 		= motorID;
		motorstate[motorID].state			= 0;
		motorstate[motorID].ref_RPM 		= 0;
		motorstate[motorID].act_RPM 		= 0;
		motorstate[motorID].set_RPM 		= 0;
		motorstate[motorID].act_Voltage 	= 0;
		motorstate[motorID].act_Current 	= 12340;
		motorstate[motorID].act_Temp 		= 0;
		motorstate[motorID].act_Power 		= 0;
		motorstate[motorID].act_Torque 		= 0;
		motorstate[motorID].abs_Rots	 	= 0;
		motorstate[motorID].ticks.rel_Ticks = 0;
		motorstate[motorID].ticks.abs_Ticks = 0;

		motorstate[motorID].old_set_rpm 	= 0;
		motorstate[motorID].setter_diff 	= 0;

		error_sys.err_motor[motorID].motor   = 0; // No Motor Error
		error_sys.err_motor[motorID].encoder = 0; // No Encoder Error
	}



}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief The Control Loop! Calls functions from the different control modules
///
/// This function executes all functions which are needed for contolling the motors and is called periodicly\n
/// It calls functions from the following moduls:\n
/// ais_error: call Error chech funtions and store results in motormotorstate.error_check struct \n
/// ais_calculatemotorstate: call functions to calculate the runtime datas\n
/// ais_controller: call the controller, a PID-Regulator at the moment, to regulate the RPMs, if applicable\n
/// ais_status: call the function to send the requested status response\n
/// \param last_cycle_length The time the last cycle used in microseconds\n
/// \note The control loop is called periodic from the main loop\n
//////////////////////////////////////////////////////////////////////////////
void main_motorcontrol(long last_cycle_length) {

	//loop counter
	VMC_INT_16 motorID = 0;


	//============================ Error check functions =====================
	// Check if there is a Command Timeout
   	error_sys.timeout = chk_timeout(last_cycle_length, get_timeout() );

	for(motorID = 0; motorID < _NUM_MOTORS_; motorID++) {

		// Check if there is any mechanical problem with the motor
        error_sys.err_motor[motorID].motor = chk_motor(motorID);

		// Check if the encodersignals are valid?
    	error_sys.err_motor[motorID].encoder = chk_encoder(motorID);
    }

	// Check if an Emergencystop is necessary?
    error_sys.emergencystop = chk_emergencystop(&error_sys);
	//============================ Error check end ===========================



	//============================ Calculating functions =====================
    for(motorID = 0; motorID < _NUM_MOTORS_; motorID++) {
		// Read the motorencoderticks
		motorstate[motorID].ticks.rel_Ticks = calc_encoderticks(motorID);

		// Accumulate the motorencoderticks
		motorstate[motorID].ticks.abs_Ticks += motorstate[motorID].ticks.rel_Ticks;

		// Calculate the actual RPM
	    motorstate[motorID].act_RPM = calc_RPM(last_cycle_length, &motorstate[motorID], get_ticks_rot(motorID) );

		// Calculate the absolute Rotations
		motorstate[motorID].abs_Rots = calc_abs_rots(&motorstate[motorID], get_ticks_rot(motorID) );

		// Calculate the motorvoltage, !not implemented yet!
		motorstate[motorID].act_Voltage = calc_voltage(motorID);

        // Read out the ADC and calculate the current
        motorstate[motorID].act_Current = calc_current(motorID);

	    // Calculate the Temperatures
     	motorstate[motorID].act_Temp = calc_temperature(last_cycle_length, &motorstate[motorID]);

		// Calculate the torques
	    motorstate[motorID].act_Torque = calc_torque(&motorstate[motorID], get_spec_torque(motorID));

	    // Calculate the powers
	    motorstate[motorID].act_Power = calc_power( &motorstate[motorID]);

	}
    //============================ Calculating functions end ================================



	//============================ The Controller ================================
	for(motorID = 0; motorID < _NUM_MOTORS_; motorID++) {

		// If controller is active calculate the new set RPM
        if ( get_controller_active(motorID) == 1) { 
 			motorstate[motorID].set_RPM = controller(motorID, motorstate[motorID].ref_RPM,
													 motorstate[motorID].act_RPM, motorstate[motorID].setter_diff,
									 		     	 get_max_RPM(motorID), last_cycle_length);	

			// Calculate next difference between controller output and setter output
			// e.g. for anti-windup
			// setter_diff is for next cycle use: set[k-1] - set[k]
			// old_set_rpm -> set[k-1] / motorstate[motorID].set_RPM -> set[k]
			motorstate[motorID].setter_diff = motorstate[motorID].old_set_rpm - motorstate[motorID].set_RPM;
		// If controller is not active simply sets the set RPM to the reference RPM
	    } else {
	        motorstate[motorID].set_RPM = motorstate[motorID].ref_RPM;
        }
	}
	//============================ The Controller end ================================




	//============================ Outputs to Motors ================================
	for(motorID = 0; motorID < _NUM_MOTORS_; motorID++) {

		// If PWM mode is not active, convert set RPM to set PWM
        if ( !get_use_PWM(motorID) ) {
            motorstate[motorID].set_PWM = convert_RPM2PWM(motorstate[motorID].set_RPM, get_max_RPM(motorID), PWM_PERIOD);
	    }

        if ( get_currentlimiter_active(motorID) ) {
            motorstate[motorID].set_PWM = current_limiter(motorstate[motorID].set_PWM);
	    }



		// If emercencystop --> PWM for all channels = 0 and deactivate controller !
	    if ( error_sys.emergencystop ) {
		     reset_controller(motorID); // Reset controller !!!
		     motorstate[motorID].set_PWM = 0;
			 motorstate[motorID].set_RPM = 0;
			 motorstate[motorID].ref_RPM = 0;
    	}

	   // Sets the set PWM, which is now definitly a PWM Signal, to the motor
       mpwr_motor_set(motorID, motorstate[motorID].set_PWM );

	   // remember last set value !
	   motorstate[motorID].old_set_rpm = motorstate[motorID].set_RPM;
	}
	//============================ Outputs to Motors End ================================


	//============================ Status and Error signals ================================
    if (error_sys.emergencystop) {
		output_msg(_OMSG_EMSTOP_);
	} else {
		output_msg(_OMSG_RUN_);
	}
	//============================ Status and Error signals End ================================

}
//////////////////////////////////////////////////////////////////////////////



//============================ Get functions ================================


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the ID of the Motor /channel [0...4]
/// \param id the id of the motor
/// \returns the ID of the Motor /channel
//////////////////////////////////////////////////////////////////////////////
unsigned char get_id(VMC_UCHAR_8 id) {
    return motorstate[id].ID;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the motorstate of the motor: 	0: acc 1: braking 2: stop
/// \param id the id of the motor
/// \returns the motorstate of the motor
//////////////////////////////////////////////////////////////////////////////
unsigned char get_motorstate(VMC_UCHAR_8 id) {
    return motorstate[id].state;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the desired RPM of the motor in RPM [-25000...25000]
/// \param id the id of the motor
/// \returns the desired RPM of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_INT_16 get_ref_RPM(VMC_UCHAR_8 id) {
    return motorstate[id].ref_RPM;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the actual RPM of the motor in RPM [-25000...25000]
/// \param id the id of the motor
/// \returns the actual RPM of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_INT_16 get_act_RPM(VMC_UCHAR_8 id) {
    return motorstate[id].act_RPM;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the control RPM of the motor in RPM [-25000..25000]
/// \param id the id of the motor
/// \returns the control RPM of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_INT_16 get_set_RPM(VMC_UCHAR_8 id) {
    return motorstate[id].set_RPM;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the control PWM of the motor in SIGNAL RANGE
/// \param id the id of the motor
/// \returns the control PWM of the motor
//////////////////////////////////////////////////////////////////////////////
int get_set_PWM(VMC_UCHAR_8 id) {
    return motorstate[id].set_PWM;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the actual current of the motor in mA [0...10000]
/// \param id the id of the motor
/// \returns the actual current of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_act_Current(VMC_UCHAR_8 id) {
    return motorstate[id].act_Current;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the actual voltage of the motor in mV [0...50000]
/// \param id the id of the motor
/// \returns the actual voltage of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_act_Voltage(VMC_UCHAR_8 id) {
    return motorstate[id].act_Voltage;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the actual temperature of the motor in celsius [0...200]
/// \param id the id of the motor
/// \returns the actual temperature of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_act_Temp(VMC_UCHAR_8 id) {
    return motorstate[id].act_Temp;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns actual power of the motor in W [0...250]
/// \param id the id of the motor
/// \returns actual power of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_act_Power(VMC_UCHAR_8 id) {
    return motorstate[id].act_Power;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the actual torque of the motor in mNm [0...10000]
/// \param id the id of the motor
/// \returns the actual torque of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_act_Torque(VMC_UCHAR_8 id) {
    return motorstate[id].act_Torque;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns absolut Rots of the motor since reset [0...2^32]
/// \param id the id of the motor
/// \returns absolut Rots of the motor since reset
//////////////////////////////////////////////////////////////////////////////
unsigned long get_abs_Rots(VMC_UCHAR_8 id) {
    return motorstate[id].abs_Rots;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns a flag if there is a mechanical problem with the motor 0: no problem 1: problem
/// \param id the id of the motor
/// \returns a flag if there is a mechanical problem with the motor
//////////////////////////////////////////////////////////////////////////////
unsigned char get_motor_Error(VMC_UCHAR_8 id) {
    return error_sys.err_motor[id].motor;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns a flag if command timeout has appeared 0: no timeout 1:timeout
/// \param id the id of the motor
/// \returns  a flag if a command timeout has appeared
//////////////////////////////////////////////////////////////////////////////
unsigned char get_timeout_Error() {
    return error_sys.timeout;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns TRUE when failure in encodersignals
/// \param id the id of the motor
/// \returns a flag if there is a problem with the encodersignals
//////////////////////////////////////////////////////////////////////////////
unsigned char get_encoder_Error(VMC_UCHAR_8 id) {
    return error_sys.err_motor[id].encoder;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns a flag if an emergencystop is necassary 0: no ES necessary 1: ES necessary
/// \param id the id of the motor
/// \returns a flag if an emergencystop is necassary
//////////////////////////////////////////////////////////////////////////////
unsigned char get_emergencystop_Error() {
    return error_sys.emergencystop;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the number of ticks in the last cycle [0...20000]
/// \param id the id of the motor
/// \returns the number of ticks in the last cycle
//////////////////////////////////////////////////////////////////////////////
VMC_INT_16 get_rel_Ticks(VMC_UCHAR_8 id) {
    return motorstate[id].ticks.rel_Ticks;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the absolute number of ticks since reset [-2^32...2^32]
/// \param id the id of the motor
/// \returns the absolute number of ticks since reset
//////////////////////////////////////////////////////////////////////////////
VMC_LONG_32 get_abs_Ticks(VMC_UCHAR_8 id) {
    return motorstate[id].ticks.abs_Ticks;
}
//////////////////////////////////////////////////////////////////////////////




//============================ Get functions End================================


///
//============================ Set functions ================================

//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the ID of the Motor /channel [0...4]
/// \param id The id of the motor
/// \param ID The id to set
//////////////////////////////////////////////////////////////////////////////
void set_id(VMC_UCHAR_8 id, VMC_UCHAR_8 ID) {
    motorstate[id].ID = ID;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets control RPM of the motor in RPM [-25000...25000]
/// \param id The id of the motor
/// \param rpm The RPM to set
//////////////////////////////////////////////////////////////////////////////
void set_set_RPM(VMC_UCHAR_8 id, VMC_INT_16 rpm) {
    motorstate[id].set_RPM = rpm;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets desired RPM of the motor in RPM [-25000...25000]
/// \param id The id of the motor
/// \param rpm The RPM to set
//////////////////////////////////////////////////////////////////////////////
void set_ref_RPM(VMC_UCHAR_8 id, VMC_INT_16 rpm) {
	if (get_direction(id) == 0) {
	    motorstate[id].ref_RPM = rpm;
	}
	else {
		motorstate[id].ref_RPM = -rpm;
	}
}
//////////////////////////////////////////////////////////////////////////////





//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the PWM to be set to the motor fpr the direct PWM mode
/// \param id The id of the motor
/// \param pwm, The PWM signal to be set
//////////////////////////////////////////////////////////////////////////////
void set_set_PWM(VMC_UCHAR_8 id, VMC_INT_16 pwm) {
	if ( get_use_PWM(id) == 1 && get_direction(id) == 1) {
	    motorstate[id].set_PWM = -pwm;
    }
	else {
		motorstate[id].set_PWM = pwm;
	}

}
//////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////////
/// \brief Set set pwm for all motors
/// This functions call the setter for the PWM Signals of the modul ais_control
/// \param pwm1 PWM Signal for channel 1 in SIGNALRANGE
/// \param pwm2 PWM Signal for channel 2 in SIGNALRANGE
/// \param pwm3 PWM Signal for channel 3 in SIGNALRANGE
//////////////////////////////////////////////////////////////////////////////
void set_all_PWM(VMC_INT_16 pwm1, VMC_INT_16 pwm2, VMC_INT_16 pwm3) {
	set_set_PWM(0, pwm1);
	set_set_PWM(1, pwm2);
	set_set_PWM(2, pwm3);
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Set reference RPM for three motors
/// This function calls the setter for the set RPMs of the modul ais_control
/// \param rpm1 RPM for motor1
/// \param rpm2 RPM for motor2
/// \param rpm3 EOM for motor3
//////////////////////////////////////////////////////////////////////////////
void set_all_RPM(VMC_INT_16 rpm1, VMC_INT_16 rpm2, VMC_INT_16 rpm3) {
	set_ref_RPM(0, rpm1);
	set_ref_RPM(1, rpm2);
	set_ref_RPM(2, rpm3);
}


/// \brief Resets a motor error for a given motor
/// \param id The id of the motor
//////////////////////////////////////////////////////////////////////////////
void reset_motor_Error(VMC_UCHAR_8 id) {
    error_sys.err_motor[id].motor = 0;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Resets the timeout error
//////////////////////////////////////////////////////////////////////////////
void reset_timeout_Error() {
    error_sys.timeout = 0;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Resets a encoder signal error for a given motor
/// \param id The id of the motor
//////////////////////////////////////////////////////////////////////////////
void reset_encoder_Error(VMC_UCHAR_8 id) {
    error_sys.err_motor[id].encoder = 0;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Resets the Emergencysto error
//////////////////////////////////////////////////////////////////////////////
void reset_emergencystop_Error() {
    error_sys.emergencystop = 0;
}
//////////////////////////////////////////////////////////////////////////////

//============================ Set functions End================================


//@}


