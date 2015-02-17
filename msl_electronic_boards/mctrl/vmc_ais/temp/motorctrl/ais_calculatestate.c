//////////////////////////////////////////////////////////////////////////////
/// \defgroup calculatestate Calculating Runtime Data
//@{
/// \file ais_calculatestate.c
///
/// \brief Reads sensors and calculate actual runtime data
/// 		
/// This module contains functions to calculate runtime parameters by reading in sensordata (motorencoder, adc)
/// and using the configuration parameter to calculate runtime data like RPM, Temperature or Power of the motor\n
/// To use the module you have to initialise it with the function init_Calculatestate()\n
/// Then you should call the different calculating functions periodic and take care to the order, because some functions need results of other ones to work properly!\n
/// Order: \n
/// 1. calc_Encoderticks(), i.e. read motorencoder\n
/// 2. calc_RPM() and calc_abs_Rots(), i.e. treat this sensor data\n
/// 3. calc_Current(), i.e read next sensor\n
/// 4. calc_Temperature(), calc_Torque(), calc_Power(), i.e. treat this densor data\n
///
/// \note Internal Motor Ids from 0 to NUM_MOTORS -1, external from 1 to NUM_MOTORS!
/// \note When you want to add a calculation function for a runtime variable please proceed as follows\n
/// 1. Add a definition and a comment of this function to the ais_calculatestate.h, one parameter should be a pointer to the MOTORSTATE struct, to have access to actual runtime data\n
/// 2. Implement the function in this file, return the calculated variable\n
/// 
/// \author Pascal Langenberg
///
/// \version 0.4
///
/// \date 09.08.2006
///
//////////////////////////////////////////////////////////////////////////////


#include "ais_calculatestate.h"
#include "ais_configdata.h"
#include "control/ais_motorctrl.h"
#include "aisc167b/ais_gpt.h"
#include "aisc167b/ais_adc.h"
#include "motor/ais_thermic.h"


// Declaration of global array for the time offset of the call of the funtion calculate_RPM in the last cycle
TMC_LONG_32 time_last[NUM_MOTORS];



//////////////////////////////////////////////////////////////////////////////
/// \brief initialization of Calculatestate Variables     
//////////////////////////////////////////////////////////////////////////////
void init_Calculatestate() {
	
	TMC_UCHAR_8 i;

	for(i = 0; i < NUM_MOTORS; i++) {
		time_last[i] = 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Read and return the Encoder Ticks since last call
/// \returns The encoderticks of the motor since last call, i.e. in the last cycle
//////////////////////////////////////////////////////////////////////////////
TMC_INT_16 calc_Encoderticks(TMC_UCHAR_8 id) {
	// Attention: Hardware access
	return get_encoderticks(id);
}
//////////////////////////////////////////////////////////////////////////////
		

//////////////////////////////////////////////////////////////////////////////
/// \brief Calculates the actual RPM with the Encoder Ticks and the time to last call of the Motor 
/// \note Only works if the Ticks per rot of the motorencoder are configured correctly!
/// \param last_cycle_length The time the last cycle used in microsseconds
/// \param motorstate A pointer to the motorstate struct of the motor
/// \returns The RPM of the motor
//////////////////////////////////////////////////////////////////////////////
TMC_LONG_32 calc_RPM(TMC_ULONG_32 last_cycle_length, struct MOTORSTATE *motorstate) {

//============================ Declarations of local variables ================================

	// local variable representing the encoderticks in the last cycle
	TMC_LONG_32 encoderticks = motorstate->ticks.rel_Ticks;

	// local variable representing the current time offset of the call of this funtion
	TMC_LONG_32 time_now;

	// local variable represnting the time since last call, not microsenconds
	TMC_UINT_16 last_time_diff_ms;

	// loacal variable to store the result of following calculations, long because of possible big values during calculation
	TMC_LONG_32 rpm;

	// local variable representing the ID od the motor
	TMC_UCHAR_8 id = motorstate->ID;

//============================ Declarations of local variables End================================


//============================ Calculate the exact time since last call ================================	
	
	// get time offest of this function call
	time_now = get_cycle_time_part();

	// With this offset and the following formula calculate the time since last call
	last_time_diff_ms = (last_cycle_length - time_last[id]) + time_now;
	
	// Converts to milliseconds
	last_time_diff_ms = last_time_diff_ms / 1000;

	// save current time offset for next cycle
	time_last[id] = time_now;

//============================ Calculate the exact time since last call End================================	


//============================ Calculate the RPM================================
	
	// calculate with formula rpm = encoderticks / time (ms) * 60000 (ms) / ticks per rot
    rpm = 60000 * encoderticks;
    rpm = rpm / last_time_diff_ms;
    rpm = rpm /  get_ticks_Rot(id);

//============================ Calculate the RPM End================================
	
	// cast to int as specified and return result
	return (int) rpm;

}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Calculate the absolute rotations of the motor since reset
/// \note Only works if the Ticks per rot of the motorencoder are configured  and the absolutes ticks are cumulated correctly!
/// \param motorstate A pointer to the motorstate struct of the motor
/// \returns The absolute rotations of the motor
//////////////////////////////////////////////////////////////////////////////
TMC_LONG_32 calc_abs_Rots(struct MOTORSTATE *motorstate) {

	// local Variable representing the result of the calculation
	TMC_LONG_32 rots;

	// local variable representing the ID od the motor
	TMC_UCHAR_8 id = motorstate->ID;

	// Calculate teh absolut rotaions with the absolute ticks and the ticks per rot
	rots = motorstate->ticks.abs_Ticks / get_ticks_Rot(id);
	
	return rots;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Calculate the actual voltage of one Motor, not yet implemented!
/// \note not yet implemented!
/// \id The id of the motor
/// \return The voltage of the motor
//////////////////////////////////////////////////////////////////////////////
TMC_UINT_16 calc_Voltage(TMC_UCHAR_8 id) {
	return id;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Reads the ADC and calculate motorcurrent
/// \id The id of the motor
/// \returns The actual Current of the motor in [mA]
//////////////////////////////////////////////////////////////////////////////
TMC_UINT_16 calc_Current(TMC_UCHAR_8 id) {

	
	//depending on the Motor id, select the corresponding ADC channel, read the value and alculate the current
    //formula: current[mA] = (adc_value * 5 / 1023) [V] / 0.56[Ohm] * 1000 (to mA)
	switch (id) {
    	case 0 :
			// Attention: Hardware access
        	return (((((float)get_sc_adc(9))*5)/1023)/0.56) *1000; 
            break;
		case 1 :
			// Attention: Hardware access
        	return (((((float)get_sc_adc(8))*5)/1023)/0.56) *1000;
            break;
        case 2 :
			// Attention: Hardware access
	        return (((((float)get_sc_adc(5))*5)/1023)/0.56) *1000;
            break;
        }

	return 0;
	
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Calculates the actual temperature of the motor
/// \param TEMP The structure which contains the parameter and actual values needed for the thermic model
/// \param last_cycle_length The time the last cycle used in microsseconds
/// \param motorstate A pointer to the motorstate struct of the motor
/// \note This funtion uses the modul ais_thermic
/// \returns actual Temperatur of the motor
//////////////////////////////////////////////////////////////////////////////
TMC_UINT_16 calc_Temperature(TMC_ULONG_32 last_cycle_length, struct MOTORSTATE *motorstate) {
	
	// local variable representing the ID of the motor
	TMC_UCHAR_8 id = motorstate->ID;

	// Call function from ais_Thermic to calculate motor Temperature
	calcMotorTemperature(id, motorstate->act_Current, last_cycle_length);

	// Call function from ais_Thermic to return motor Winding Temperature and return result
	return getMotorWindingTemperature(id);
	
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Calculates the actual torque on one Motor 
/// \param motorstate A pointer to the motorstate struct of the motor
/// \returns the actual torque of the motor
//////////////////////////////////////////////////////////////////////////////
TMC_UINT_16 calc_Torque(struct MOTORSTATE *motorstate) {
	// forumula: torque = current [A] * speciique torqe[mNm/A] = mNm, / 1000 because of using mA
    return motorstate->act_Current * get_spec_Torque(motorstate->ID) / 1000;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Calculates the actual Power on one Motor 
/// \param motorstate A pointer to the motorstate struct of the motor
/// \returns the actual power of the motor
//////////////////////////////////////////////////////////////////////////////
TMC_UINT_16 calc_Power(struct MOTORSTATE *motorstate) {
    return 0;
}
//////////////////////////////////////////////////////////////////////////////
//@}
