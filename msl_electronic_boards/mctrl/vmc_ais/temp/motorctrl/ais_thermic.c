//////////////////////////////////////////////////////////////////////////////
/// \defgroup thermic Thermic Modul
//@{
/// \file ais_thermic.c
///
/// \brief Thermic Modul
/// 
/// \author Jan Paulus and Dipl. Ing. Kubina
///
/// \version 0.8
///
/// \date 08.08.2005
///
//////////////////////////////////////////////////////////////////////////////

#include "motor/ais_thermic.h"
#include "control/ais_configdata.h"


TMC_UINT_16 deltaWindingTemperatur[NUM_MOTORS];
TMC_UINT_16 deltaChassisTemperatur[NUM_MOTORS];
TMC_UINT_16 ENVIRONMENT_TEMP;

//----------- Konstanten der Übertragungsfunktion Winding --------
// PT1 Glied         
//Out1 / In1 =  (GAIN_ZAELER / GAIN_NENNER)*1/(1 + s*TIME_CONSTANT)
    // 1 ... 500 s
TMC_UINT_16 TIME_CONSTANT_WINDING[NUM_MOTORS];
    //1... 100
TMC_UINT_16 GAIN_NENNER_WINDING[NUM_MOTORS];
    //1... 1000
TMC_UINT_16 GAIN_ZAELER_WINDING[NUM_MOTORS];
//----------- Konstanten der Übertragungsfunktion Winding --------
// PT1 Glied         
    // 1 ... 500 s
TMC_UINT_16 TIME_CONSTANT_CHASSIS[NUM_MOTORS];
    //1... 100
TMC_UINT_16 GAIN_NENNER_CHASSIS[NUM_MOTORS];
    //1... 1000
TMC_UINT_16 GAIN_ZAELER_CHASSIS[NUM_MOTORS];
//------------------- Additional Variables ---------------------
    //Wertebereich von oldOut,out  +- In1[0]*GAIN_ZAELER/GAIN_NENNER
//uword oldOut_Winding[NUM_MOTOR];
    //         !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //Wertebereich von summe   +- In1[0]*GAIN_ZAELER/GAIN_NENNER*1000/DeltaTIME*TIME_CONSTANT + 10%
    // long ca.+- 2*10^9
TMC_ULONG_32 summe_Winding[NUM_MOTORS];
TMC_ULONG_32 summe_Chassis[NUM_MOTORS];

TMC_UINT_16 MIN_Temperatur[NUM_MOTORS];//°C
TMC_UINT_16 MAX_Temperatur[NUM_MOTORS];//°C
TMC_UINT_16 actSwitchState[NUM_MOTORS];

TMC_UINT_16 nominal_current[NUM_MOTORS];
TMC_UINT_16 max_current[NUM_MOTORS];


//////////////////////////////////////////////////////////////////////////////
/// \brief Initialsation of the Variables for the Thermic Modul     
//////////////////////////////////////////////////////////////////////////////
void init_Thermic(void) {
TMC_UCHAR_8 i;

	for(i = 0; i < NUM_MOTORS; i++) {
		
		summe_Winding[i] = 0;
		summe_Chassis[i] = 0;

	
		MIN_Temperatur[i] = 0;
		MAX_Temperatur[i] = 0;
		actSwitchState[i] = 0;// 0 = off;

		nominal_current[i] = 0;
		max_current[i] = 0;

		set_WindingConstants(i, get_winding_t(i), get_winding_g_n(i), get_winding_g_z(i));
		set_ChassisConstants(i, get_chassis_t(i), get_chassis_g_n(i), get_chassis_g_z(i));
		setThermicHysteresisSwitchConstants(i, get_nom_Temp(i), get_max_Temp(i));
		setCurrentValues(i, get_nom_Current(i), get_max_Current(i));
	}
	setEnvironment_Temperature(get_env_Temp(0));

	return;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief returns the Temperature of the Winding      
/// \param Motor_ID Motor number
/// \returns Winding Temperature
//////////////////////////////////////////////////////////////////////////////
TMC_UINT_16 getMotorWindingTemperature( unsigned char Motor_ID){
	return (deltaWindingTemperatur[Motor_ID] + deltaChassisTemperatur[Motor_ID] + ENVIRONMENT_TEMP);
}

//////////////////////////////////////////////////////////////////////////////
/// \brief returns the Temperature of the Chassis      
/// \param Motor_ID Motor number
/// \returns Chassis Temperature
//////////////////////////////////////////////////////////////////////////////
TMC_UINT_16 getMotorChassisTemperature(TMC_UCHAR_8 Motor_ID){
	return (deltaChassisTemperatur[Motor_ID] + ENVIRONMENT_TEMP);
}


//////////////////////////////////////////////////////////////////////////////
/// \brief returns the State of the thermic hysteresis     
/// \param Motor_ID Motor number
/// \returns 0 for off (temperatur under limit; heating phase)
/// \returns 1 for on (temperatur to high; cool-down phase)
//////////////////////////////////////////////////////////////////////////////
TMC_UCHAR_8 getThermicHysteresisSwitchState( TMC_UCHAR_8 Motor_ID){
	return actSwitchState[Motor_ID];
}


//////////////////////////////////////////////////////////////////////////////
/// \brief sets the Environment Temperature     
/// \param ENV_TEMP Environment Temperature
//////////////////////////////////////////////////////////////////////////////
void setEnvironment_Temperature(const TMC_UINT_16 ENV_TEMP){
   if(ENV_TEMP <= 100){
   		ENVIRONMENT_TEMP = ENV_TEMP;
   }


}

//////////////////////////////////////////////////////////////////////////////
/// \brief sets the thermic constants for the winding to chassis 
/// PT1 Glied         
/// Out / In =  (G_Z / G_N)*1/(1 + s * T)
/// 1 ...T... 500 s 
/// 1 ...G_Z... 1000
/// 1 ...N_Z... 100    
/// \param Motor_ID Motor number
/// \param T time constants [ms]
/// \param G_N denominator of the thermic resistor [1/KW]
/// \param G_Z numerator of the thermic resistor [1/KW]
//////////////////////////////////////////////////////////////////////////////
void set_WindingConstants(TMC_UCHAR_8 Motor_ID, TMC_UINT_16 T, TMC_UINT_16 G_N, TMC_UINT_16 G_Z){
	if( (Motor_ID >= 0)&&(Motor_ID < NUM_MOTORS)){
	    if( (T > 0)&&(T < 500))
			TIME_CONSTANT_WINDING[Motor_ID] = T;
	    if( (G_N > 0)&&(G_N < 100))
			GAIN_NENNER_WINDING[Motor_ID] = G_N;
	    if( (G_Z > 0)&&(G_Z < 1000))
			GAIN_ZAELER_WINDING[Motor_ID] = G_Z;
	}
	
}

//////////////////////////////////////////////////////////////////////////////
/// \brief sets the thermic constants for the chassis to the Environment
/// PT1 Glied         
/// Out / In =  (G_Z / G_N)*1/(1 + s * T)
/// 1 ...T... 500 s 
/// 1 ...G_Z... 1000
/// 1 ...N_Z... 100  
/// \param Motor_ID Motor number
/// \param T time constants [ms]
/// \param G_N denominator of the thermic resistor [1/KW]
/// \param G_Z numerator of the thermic resistor [1/KW]
//////////////////////////////////////////////////////////////////////////////
void set_ChassisConstants(TMC_UCHAR_8 Motor_ID, TMC_UINT_16 T, TMC_UINT_16 G_N, TMC_UINT_16 G_Z){
	if( (Motor_ID >= 0)&&(Motor_ID < NUM_MOTORS)){
	    if( (T > 0)&&(T < 500))
			TIME_CONSTANT_CHASSIS[Motor_ID] = T;
	    if( (G_N > 0)&&(G_N < 100))
			GAIN_NENNER_CHASSIS[Motor_ID] = G_N;
	    if( (G_Z > 0)&&(G_Z < 1000))
			GAIN_ZAELER_CHASSIS[Motor_ID] = G_Z;
	}
	
}

//////////////////////////////////////////////////////////////////////////////
/// \brief sets the Temperatures for the Hysteresis Switch
/// \param Motor_ID Motor number
/// \param MIN_Temp temperature when the motor starts to overheat [°C]
/// \param MAX_Temp absulut maximum temperature of the motor [°C]
//////////////////////////////////////////////////////////////////////////////
void setThermicHysteresisSwitchConstants( TMC_UCHAR_8 Motor_ID, TMC_UINT_16 MIN_Temp, TMC_UINT_16 MAX_Temp){
	if( (Motor_ID >= 0)&&(Motor_ID < NUM_MOTORS)){
		MIN_Temperatur[Motor_ID] =  MIN_Temp; 
		MAX_Temperatur[Motor_ID] =  MAX_Temp; 
	}


}

//////////////////////////////////////////////////////////////////////////////
/// \brief sets the current values for the Temperature calculation
/// \param Motor_ID Motor number
/// \param c_nominal nominal Current of the motor [mA]
/// \param c_max maximum Current of the motor [mA]
//////////////////////////////////////////////////////////////////////////////
void setCurrentValues( TMC_UCHAR_8 Motor_ID, TMC_UINT_16 c_nominal, TMC_UINT_16 c_max) {
	if( (Motor_ID >= 0)&&(Motor_ID < NUM_MOTORS)){
		nominal_current[Motor_ID] = c_nominal;
		max_current[Motor_ID] = c_max;
	}
	
}


//////////////////////////////////////////////////////////////////////////////
/// \brief calculates the Winding and Chassis Temperature
/// \param Motor_ID Motor number
/// \param actCurrent actual Current in this moment [mA]
/// \param DeltaTIME Time since last call
//////////////////////////////////////////////////////////////////////////////
TMC_UINT_16 calcMotorTemperature(const TMC_UCHAR_8 Motor_ID, TMC_UINT_16 actCurrent,TMC_UINT_16 DeltaTIME){
    TMC_ULONG_32 Ifactor = 0;
	TMC_ULONG_32 square = 0;
	TMC_ULONG_32 out = 0;
	TMC_LONG_32 e = 0;
	if( (Motor_ID < NUM_MOTORS)&&(Motor_ID >= 0) ){
	 	 //---------- Faktorize Iact -------
		 // wobei 8 = Inenn entspricht
	
		
		 Ifactor = actCurrent << 3;
		 Ifactor = Ifactor / nominal_current[Motor_ID];
		  Ifactor -= 1;	//to have a faster cooling at Inenn
		 //---------- Square ----------
		 square = Ifactor * Ifactor;
		 square = square >> 3;
	//printf("square:%ld \n  ", square);
		 //---------- PT1 Winding ---------
		 e = square * GAIN_ZAELER_WINDING[Motor_ID];
		 e /= GAIN_NENNER_WINDING[Motor_ID];
		 e -= deltaWindingTemperatur[Motor_ID];

		 summe_Winding[Motor_ID] = summe_Winding[Motor_ID] + e;
		 out = summe_Winding[Motor_ID];
	//printf("summ%d :%ld \n", Motor_ID, summe_Winding[Motor_ID]);
		 out = out * DeltaTIME;
		 out = out / TIME_CONSTANT_WINDING[Motor_ID];
		 out = out / 1000; // to [ms]

		 deltaWindingTemperatur[Motor_ID] = out;
    //printf("Current:%d   ",actCurrent );
	//printf("dW:%ld   ", out);
		 //----------in row PT1 Chassis ---------
		 e = deltaWindingTemperatur[Motor_ID] * GAIN_ZAELER_CHASSIS[Motor_ID];
		 e /= GAIN_NENNER_CHASSIS[Motor_ID];
		 e -= deltaChassisTemperatur[Motor_ID];

		 summe_Chassis[Motor_ID] = summe_Chassis[Motor_ID] + e;
		 out = summe_Chassis[Motor_ID];
		 out = out / DeltaTIME;
		 out = out / TIME_CONSTANT_CHASSIS[Motor_ID];
		 out = out / 1000; // to [ms]
		 
		 deltaChassisTemperatur[Motor_ID] = out;
		 //printf("dCh:%ld\n", out);
        return out;
	
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Thermic Hysteresis Switch
/// \param Motor_ID Motor number
/// \param actTemperatur actual Temperature
/// \returns maximum current if the actual Temperature hadn't reached the maximum Temperature
/// \returns nominal current if the maximum Temperature is reached and as long the Temperature is still over the MIN_Temperatue
//////////////////////////////////////////////////////////////////////////////
TMC_UINT_16 calcThermicHysteresisSwitch(TMC_UCHAR_8 Motor_ID, TMC_UINT_16 actTemperatur){
	TMC_UINT_16 actOutput = 0;
	//-------------- rising Temperature over MAX ------------------------
    if((actTemperatur > MAX_Temperatur[Motor_ID]) && (actSwitchState[Motor_ID] == 0) ){	
		actSwitchState[Motor_ID] = 1; // 1 = on
	}
    if((actTemperatur < MIN_Temperatur[Motor_ID]) && (actSwitchState[Motor_ID] == 1)){
		actSwitchState[Motor_ID] = 0; // 0 = off	
	}

	if(actSwitchState[Motor_ID] == 1){
		actOutput = nominal_current[Motor_ID];
	}else{
		actOutput = max_current[Motor_ID];
	}
	return actOutput;
}


//////////////////////////////////////////////////////////////////////////////
/// \brief combines all function above to one
/// \param Motor_ID Motor number
/// \param actual_current actual Current in this moment [mA]
/// \param max_current return what is the maximum current at the moment
/// \param actual_temp return the actual Temperature
/// \param DeltaTIME Time since last call
//////////////////////////////////////////////////////////////////////////////
void Thermic( TMC_UCHAR_8 Motor_ID, TMC_UINT_16 actual_current, TMC_UINT_16* max_current, TMC_UINT_16* actual_temp, const TMC_UINT_16 DeltaTIME ) {

	calcMotorTemperature(Motor_ID, actual_current, DeltaTIME);
	*max_current = calcThermicHysteresisSwitch(Motor_ID, getMotorWindingTemperature(Motor_ID) );
	*actual_temp = getMotorWindingTemperature(Motor_ID);
}

//@}


