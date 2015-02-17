//////////////////////////////////////////////////////////////////////////////
/// \defgroup limit Current Limiter
//@{
/// \file ais_limit.c
///
/// \brief Current Limiter function
/// 		
/// \note all Currents are in milli Ampere and positiv at all times
/// 
/// \author Jan Paulus and Pascal Langenberg
///
/// \version 0.9
///
/// \date 08.08.2006
///
//////////////////////////////////////////////////////////////////////////////

#include "aisc167b/ais_gpt.h"
#include "motor/ais_limit.h"
#include "control/ais_motorctrl.h"
#include "control/ais_configdata.h"

long Summ[NUM_MOTORS];
unsigned int L_TN[NUM_MOTORS];
unsigned char state[NUM_MOTORS];
long time_last_limit[NUM_MOTORS];


//////////////////////////////////////////////////////////////////////////////
/// \brief initialization of Limiter Variables     
//////////////////////////////////////////////////////////////////////////////
void init_Limiter(unsigned char ID, unsigned int tn) {

	L_TN[ID] = tn;
	Summ[ID] = 0;
	state[ID] = 0;
	time_last_limit[ID] = 0;
	return;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief returns if the current limiter is active     
//////////////////////////////////////////////////////////////////////////////
unsigned char get_Limiter_state(unsigned char ID) {
 	return state[ID];
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Limits the current to the maximum Current
/// \brief a integrate regulator is used    
/// \note the Regulator Variables is set by the init_Limiter() function call in the init.c file
/// \note the incoming Signal will only be modified if the actual current is bigger than the maximum current
/// \note otherwise the signal will be returns unmodified
/// \param Motor_ID Motor number
/// \param inValue the Signal to the motors (like the PWM Signal)
/// \param max_current the current value to limit to in [mA]
/// \param actual_current actual current in [mA]
/// \param DeltaTIME Time since last call in [µs]
/// \returns new control value for the motor
//////////////////////////////////////////////////////////////////////////////
int CurrentLimitter(unsigned char Motor_ID, int inValue, int max_current, int actual_current, long DeltaTIME){
  long e = 0;
	int w = 0; 
	long temp;
	long time_now;

	// calculate the exact time since last call
	time_now = get_cycle_time_part();
	DeltaTIME = (DeltaTIME - time_last_limit[Motor_ID]) + time_now;
	time_last_limit[Motor_ID] = time_now;

	if(Motor_ID < 0 || Motor_ID >= NUM_MOTORS)
		return 0;

	// Normilize the current Signal to SIGNAL_RANGE
	e = ((long)(max_current - actual_current)*get_max_RPM(Motor_ID))/ABSOLUTEMAX_CURRENT;
	
	// Integrator Regulator
	Summ[Motor_ID] = ((e * DeltaTIME)/1000) + Summ[Motor_ID];
		
		// Anti Windup
		temp = get_max_RPM(Motor_ID) * (long)L_TN[Motor_ID];
	  if(Summ[Motor_ID] < -temp){
	  	Summ[Motor_ID] = -temp;
	  }
	  if(Summ[Motor_ID] > temp){
	  	Summ[Motor_ID] = temp;
	  }		 
 	// part of the integrator Regulator
	w	 = (int)(Summ[Motor_ID] / L_TN[Motor_ID]);
	
		
	// new Signal have to be also negative if incoming Signal is negative
	if(inValue < 0)
		w =-w;
	
	// checks if incoming signal is smaller than Current limiter Signal or current is small enough
	// and disables the Current Limiter
	if( (inValue < 0 && inValue > w ) || (inValue >= 0 && inValue < w )){ 
		state[Motor_ID] = 0;
		return inValue;
	}	

	// the limiter is active	
	state[Motor_ID] = 1;
	return w;
}

//@}


