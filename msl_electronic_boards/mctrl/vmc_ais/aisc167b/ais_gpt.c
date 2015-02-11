//////////////////////////////////////////////////////////////////////////////
/// \defgroup gpt General Purpose Time Units
//@{
/// \file ais_gpt.c
///
/// \brief 	Driver Functions for GPT on the AISC167Board
///
/// \author Jan Paulus
///
/// \version 0.8
///
/// \date 24.08.2005
///
///
///
//////////////////////////////////////////////////////////////////////////////

#include <reg167.h>
#include "aisc167b/ais_gpt.h"

//////////////////////////////////////////////////////////////////////////////
/// \brief Initialisation of Timer6 for the Cycle Time measurement
///	\note the timer will have a overflow every 52.5ms
//////////////////////////////////////////////////////////////////////////////
void init_cycletime_counter(void){

	//T6CON = 0x0002;		// T6, F=1.25MHz, Resolution 800ns, T=52.5ms, Timer Mode, Count Up
	T6CON = 0x0003;			// T6, F= 625kHz, Resolution 1,6us, T=105ms, Timer Mode, Count Up
	T6 = 0;					// Start T6 from Zero
	T6R = 1;				// Start Timer
}


//////////////////////////////////////////////////////////////////////////////
/// \brief Measures the time to the last call (cycle time)
///	\note the timer will have a overflow every 52.5ms 
/// \return time difference between now and last call in [µs]
/// \return 0 if a overflow occurrence
//////////////////////////////////////////////////////////////////////////////
VMC_LONG_32 get_cycle_time(void){
	VMC_LONG_32 now;

	now = T6;
	now = (now << 3) / 5;
	if(T6OTL)
		now = 0;

	T6 = 0;
	return now ;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief return the time the cycle is already running but do not reset the timer
//////////////////////////////////////////////////////////////////////////////
VMC_LONG_32 get_cycle_time_part(void){
	return ((long)T6 << 3) / 5;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Initialisation of Timer2, Timer3, Timer4 to count the Encoder ticks
/// \param motor_id number of motor for which you need the encoder for
//////////////////////////////////////////////////////////////////////////////
void init_encoder(VMC_UCHAR_8 motor_id){

	switch(motor_id) {
		case 0:
			T2CON = 0x0189;   // Counter2, T2UDE=1, T2UD=1, Counter Mode, rising edge
			dp_T2IN = 0;			// init Direction register
			T2 = half;				// Start Counter from half
			T2R = 1;					// Start Counter 2
		break;
		case 1:
			T3CON = 0x0189;   // Counter3, T3UDE=1, T3UD=1, Counter Mode, rising edge
			dp_T3EUD = 0;			// init Direction register
			dp_T3IN = 0;			// init Direction register
			T3 = half;				// Start Counter from half
			T3R = 1;					// Start Counter 3
		break;
		case 2:
			T4CON = 0x0189;   // Counter4, T4UDE=1, T4UD=1, Counter Mode, rising edge
			dp_T4IN = 0;			// init Direction register
			T4 = half;				// Start Counter from half
			T4R = 1;					// Start Counter 4
		break;
	}
}

//////////////////////////////////////////////////////////////////////////////
/// \brief gets the number of ticks between now and the last call
/// \note you cann calculate when the counter will have a overflow with this fromel 
/// \note overflow_time = 32768/((max_speed in sec) * ticks_per_rot)   Example: 32768/((7580/60)*500) = 0,519 sec
/// \param id number of motor
/// \return number of ticks form the encoder (positiv or negative depends on the direction of the motor)
//////////////////////////////////////////////////////////////////////////////
VMC_LONG_32 get_encoderticks(VMC_UCHAR_8 id){
	VMC_LONG_32 Counter;

	switch(id) {
		case 0:
			Counter = T2;
			T2 = half;
			return (half - Counter);
		break;
		case 1:
			Counter = T3;
			T3 = half;
			return (half - Counter);
		break;
		case 2:
			Counter = T4;
			T4 = half;
			return (half - Counter);
		break;
	}
	return 0;
}


	

//@}
