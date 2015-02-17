//////////////////////////////////////////////////////////////////////////////
/// \defgroup configdata Configuration Data
//@{
/// \file ais_motorconfig.c
///
/// \brief Configuration parameter and corresponding set/get functions
///
/// This module contains the data for the configuration of the motors connected to the TMC 2000 and the TMC itself. \n
/// For any configuration parameter there exists one set function and one get funtion to change and read the corresponding parameter.
/// To use this module you have to initialise it with the function init_Configdata() and then use the data via the set/get functions
///
/// \note To add Configuration Parameter you have to to the following steps: \n
///  1. Define and comment the parameter in the ais_configdata.h \n
///  2. Define in .h and implement in .c  get/set functions, use existing ones as templates \n
///  3. Integrate parameter in the ais_motorcmd module to make it configurable via communication channel, see in the module how to do  \n
///  4. Integrate parameter in the ais_configsave module to make it storable, see in the module how to do \n
///
/// \author Pascal Langenberg, Adam Cwientzek
///
/// \version 0.5
///
/// \date 02.10.2006
///
//////////////////////////////////////////////////////////////////////////////

#include "ais_typedef.h"
#include "motorctrl/ais_configmanager.h"
#include "motorctrl/ais_motorconfig.h"


struct MOTORCONFIG motor[_NUM_MOTORS_];
struct VMCCONFIG vmcconfig;




//////////////////////////////////////////////////////////////////////////////
/// \brief Load the default config
//////////////////////////////////////////////////////////////////////////////
void set_default_motorconfig() {
    VMC_UCHAR_8 i; //loop counter

    for(i = 0; i < _NUM_MOTORS_;i++) {
		set_max_current(i, 			4000);
		set_max_RPM(i, 				12000);
		set_nom_current(i, 			2500);
		set_nom_RPM(i, 				10000);
		set_max_temp(i, 			150);
		set_nom_temp(i, 			100);
		set_env_temp(i, 			20);
		set_winding_t(i, 			0);
		set_winding_g_z(i, 			0);
		set_winding_g_n(i, 			0);
		set_chassis_t(i, 			0);
		set_chassis_g_z(i, 			0);
		set_chassis_g_n(i, 			0);
		set_direction(i,			0);
		set_gear_reduction(i, 		74);
		set_wheel_radius(i, 		125);

		set_ticks_rot(i, 			500);
		set_spec_torque(i, 			39);

		set_currentlimiter_active(i, 0);
	    set_controller_active(i, 	1);
	    set_use_PWM(i, 				0);   ///@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 0
		set_timeout(				100);

	}
}

VMC_INT_16 convert_RPM2PWM(VMC_INT_16 rpm, VMC_INT_16 max_rpm, VMC_INT_16 pwm_period ) {
    long pwm = ((long)rpm * (long)pwm_period) / (long) max_rpm;
    return (int)pwm;
}

VMC_INT_16 convert_PWM2RPM(VMC_INT_16 pwm, VMC_INT_16 max_rpm, VMC_INT_16 pwm_period ) {
	long rpm = ((long)pwm * (long)max_rpm) / (long) pwm_period;
    return (int)rpm;
}

//============================ get functions ====================================


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns a pointer to the TEMP structure of one motor for external use of theses data
/// \param id The id of the motor
/// \returns a pointer to the TEMP structure
//////////////////////////////////////////////////////////////////////////////
struct TEMP *get_temp_params(VMC_UCHAR_8 id) {
    return &motor[id].temp;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the maximum allowed current of the motor in mA[0...10000]
/// \param id The id of the motor
/// \returns the maximum allowed current of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_max_current(VMC_UCHAR_8 id) {
    return motor[id].max_Current;
}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the maximum specified RPM of the motor in RPM [0...25000]
/// \param id The id of the motor
/// \returns the maximum specified RPM of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_max_RPM(VMC_UCHAR_8 id) {
    return motor[id].max_RPM;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the nominal current of the motor in mA[0...8000]
/// \param id The id of the motor
/// \returns the nominal current of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_nom_current(VMC_UCHAR_8 id) {
    return motor[id].nom_Current;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the nominal RPM of the motor in RPM [0...25000]
/// \param id The id of the motor
/// \returns the nominal RPM of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_nom_RPM(VMC_UCHAR_8 id) {
    return motor[id].nom_RPM;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the maximum temperature of the motor in celsius [0...200]
/// \param id The id of the motor
/// \returns the maximum temperature of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_max_temp(VMC_UCHAR_8 id) {
    return motor[id].temp.max_Temp;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the nominal temperatur of the motor in celsius [0...200]
/// \param id The id of the motor
/// \returns the nominal temperatur of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_nom_temp(VMC_UCHAR_8 id) {
    return motor[id].temp.nom_Temp;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the environment temperatur of the motor in celsius [0...100]
/// \param id The id of the motor
/// \returns the environment temperatur of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_env_temp(VMC_UCHAR_8 id) {
    return motor[id].temp.env_Temp;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns time constant for the winding of the motor in ms[0...500]
/// \param id The id of the motor
/// \returns time constant for the winding of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_winding_t(VMC_UCHAR_8 id) {
    return motor[id].temp.winding_t;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns nominator of the thermic resistor of the winding of the motor in 1/KW [1...1000]
/// \param id The id of the motor
/// \returns nominator of the thermic resistor of the winding of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_winding_g_z(VMC_UCHAR_8 id) {
    return motor[id].temp.winding_g_z;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns denominator of the thermic resistor of the winding in 1/KW [1...100]
/// \param id The id of the motor
/// \returns denominator of the thermic resistor of the winding
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_winding_g_n(VMC_UCHAR_8 id) {
    return motor[id].temp.winding_g_n;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns time constant for the chassis of the motor in ms[0...500]
/// \param id The id of the motor
/// \returns time constant for the chassis of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_chassis_t(VMC_UCHAR_8 id) {
    return motor[id].temp.chassis_t;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns nominator of the thermic resistor of the chassis in 1/KW [1...1000]
/// \param id The id of the motor
/// \returns nominator of the thermic resistor of the chassis
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_chassis_g_z(VMC_UCHAR_8 id) {
    return motor[id].temp.chassis_g_z;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns denominator of the thermic resistor of the chassis in 1/KW [1...100]
/// \param id The id of the motor
/// \returns denominator of the thermic resistor of the chassis
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_chassis_g_n(VMC_UCHAR_8 id) {
    return motor[id].temp.chassis_g_n;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the gear reduction of the motor in rots motor/wheel * 10 [1...10000]
/// \param id The id of the motor
/// \returns the gear reduction of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_gear_reduction(VMC_UCHAR_8 id) {
    return motor[id].hardware.gear_Reduction;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns wheel radius of the motor in mm[1...1000]
/// \param id The id of the motor
/// \returns wheel radius of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_wheel_radius(VMC_UCHAR_8 id) {
    return motor[id].hardware.wheel_Radius;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Returns encoder ticks per rot of the motor[1...5000]
/// \param id The id of the motor
/// \returns encoder ticks per rot of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_ticks_rot(VMC_UCHAR_8 id) {
    return motor[id].hardware.ticks_Rot	;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns The specifique torque of the motor in mNm/A [1...1000]
/// \param id The id of the motor
/// \returns The specifique torque of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_spec_torque(VMC_UCHAR_8 id) {
    return motor[id].hardware.spec_Torque;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns The specified direction of the motor
/// \param id The id of the motor
/// \returns The specified direction of the motor
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_direction(VMC_UCHAR_8 id) {
    return motor[id].hardware.direction;
}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the status of the controller (Regulator): 1:active 0:inactive
/// \param id the id of the motor
/// \returns the status of the controller
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 get_controller_active(VMC_UCHAR_8 id) {
	return motor[id].controller.controller_active;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the type of speed input: 1:pwm 0:RPM
/// \param id the id of the motor
/// \returns the type of speed input
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 get_use_PWM(VMC_UCHAR_8 id) {
	return motor[id].controller.use_PWM;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief he status of the Current Limiter: 1:active 0:inactive
/// \param id the id of the motor
/// \returns the status of the current limiter
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 get_currentlimiter_active(VMC_UCHAR_8 id) {
	return motor[id].controller.current_limiter_active;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Returns the Timeout of the controller in cycles
/// \param id the id of the motor
/// \returns the timeout
//////////////////////////////////////////////////////////////////////////////
VMC_UINT_16 get_timeout() {
	return vmcconfig.timeout;
}
///////////////////////////////////////////////////////////////////////////


//============================ get functions end ====================================


//============================ set functions ====================================


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the maximum allowed current of the motor in mA[0...10000]
/// \param id The id of the motor
/// \param current The current to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_max_current(VMC_UCHAR_8 id, VMC_UINT_16 current) {
	if (id >= 0 && id < _NUM_MOTORS_ && current > 0 && current <= _MAX_CUR_) {
	    	motor[id].max_Current = current;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the maximum specified RPM of the motor in RPM [0...25000]
/// \param id The id of the motor
/// \param rpm The RPM to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_max_RPM(VMC_UCHAR_8 id, VMC_UINT_16 rpm) {
     if (id >= 0 && id < _NUM_MOTORS_ && rpm > 0 && rpm <= _MAX_RPM_) {
	    motor[id].max_RPM = rpm;
		return 1;
	}
	else {
		return 0;
	};
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the nominal current of the motor in mA[0...8000]
/// \param id The id of the motor
/// \param current The current to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_nom_current(VMC_UCHAR_8 id, VMC_UINT_16 current) {
     	if (id >= 0 && id < _NUM_MOTORS_ && current > 0 && current <= _NOM_CUR_) {
	    motor[id].nom_Current = current;
		return 1;
	}
	else {
		return 0;
	};
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the nominal RPM of the motor in RPM [0...25000]
/// \param id The id of the motor
/// \param rpm The RPM to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_nom_RPM(VMC_UCHAR_8 id, VMC_UINT_16 rpm) {
    if (id >= 0 && id < _NUM_MOTORS_ && rpm > 0 && rpm <= _NOM_RPM_) {
	    motor[id].nom_RPM = rpm;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the maximum temperature of the motor in celsius [0...200]
/// \param id The id of the motor
/// \param temp The Temperature to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_max_temp(VMC_UCHAR_8 id, VMC_UINT_16 temp) {
	if (id >= 0 && id < _NUM_MOTORS_ && temp > 0 && temp <= _MAX_TEMP_) {
	    motor[id].temp.max_Temp = temp;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the nominal temperatur of the motor in celsius [0...200]
/// \param id The id of the motor
/// \param temp The Temperature to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_nom_temp(VMC_UCHAR_8 id, VMC_UINT_16 temp) {
     if (id >= 0 && id < _NUM_MOTORS_ && temp > 0 && temp <= _NOM_TEMP_) {
	    motor[id].temp.nom_Temp = temp;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the environemnet temperatur of the motor in celsius [0...100]
/// \param id The id of the motor
/// \param temp The Temperature to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_env_temp(VMC_UCHAR_8 id, VMC_UINT_16 temp) {
     if (id >= 0 && id < _NUM_MOTORS_ && temp > 0 && temp <= _ENV_TEMP_) {
	    motor[id].temp.env_Temp = temp;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the time constant for the winding of the motor in ms[0...500]
/// \param id The id of the motor
/// \param t The time constant value to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_winding_t(VMC_UCHAR_8 id, VMC_UINT_16 t) {
    if (id >= 0 && id < _NUM_MOTORS_ && t > 0 && t <= _WINDING_T_) {
	    motor[id].temp.winding_t = t;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the nominator of the thermic resistor of the winding of the motor in 1/KW [1...1000]
/// \param id The id of the motor
/// \param gz The thermic resistor value nominator to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_winding_g_z(VMC_UCHAR_8 id, VMC_UINT_16 gz) {
    if (id >= 0 && id < _NUM_MOTORS_ && gz > 0 && gz <= _WINDING_GZ_) {
	    motor[id].temp.winding_g_z = gz;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the denominator of the thermic resistor of the winding in 1/KW [1...100]
/// \param id The id of the motor
/// \param gn The thermic resistor value denominator to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_winding_g_n(VMC_UCHAR_8 id, VMC_UINT_16 gn) {
     if (id >= 0 && id < _NUM_MOTORS_ && gn > 0 && gn <= _WINDING_GN_) {
	    motor[id].temp.winding_g_n = gn;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the time constant for the chassis of the motor in ms[0...500]
/// \param id The id of the motor
/// \param t The time constant value to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_chassis_t(VMC_UCHAR_8 id, VMC_UINT_16 t) {
    if (id >= 0 && id < _NUM_MOTORS_ && t > 0 && t <= _CHASSIS_T_) {
	    motor[id].temp.chassis_t = t;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the nominator of the thermic resistor of the chassis in 1/KW [1...1000]
/// \param id The id of the motor
/// \param gz The thermic resistor value nominator to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_chassis_g_z(VMC_UCHAR_8 id, VMC_UINT_16 gz) {
      if (id >= 0 && id < _NUM_MOTORS_ && gz > 0 && gz <= _CHASSIS_GZ_) {
	    motor[id].temp.chassis_g_z = gz;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the denominator of the thermic resistor of the chassis in 1/KW [1...100]
/// \param id The id of the motor
/// \param gn The thermic resistor value denominator to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_chassis_g_n(VMC_UCHAR_8 id, VMC_UINT_16 gn) {
      if (id >= 0 && id < _NUM_MOTORS_ && gn > 0 && gn <= _CHASSIS_GN_) {
	    motor[id].temp.chassis_g_n = gn;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the the gear ratio of the motor in rots motor/wheel * 10 [1...10000]
/// \param id The id of the motor
/// \param reduction The gear reduction value to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_gear_reduction(VMC_UCHAR_8 id, VMC_UINT_16 reduction) {
    if (id >= 0 && id < _NUM_MOTORS_ && reduction > 0 && reduction <= _GEAR_RATIO_) {
	    motor[id].hardware.gear_Reduction = reduction;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the wheel radius of the motor in mm[1...1000]
/// \param id The id of the motor
/// \param radius The radius to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_wheel_radius(VMC_UCHAR_8 id, VMC_UINT_16 radius) {
     if (id >= 0 && id < _NUM_MOTORS_ && radius > 0 && radius <= _WHEEL_RADIUS_) {
	    motor[id].hardware.wheel_Radius = radius;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the encoder ticks per rot of the motor[1...5000]
/// \param id The id of the motor
/// \param ticks The ticks value to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_ticks_rot(VMC_UCHAR_8 id, VMC_UINT_16 ticks) {
    if (id >= 0 && id < _NUM_MOTORS_ && ticks > 0 && ticks <= _TICKS_ROT_) {
	    motor[id].hardware.ticks_Rot = ticks;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the specifique torque of the motor in mNm/A [1...1000]
/// \param id The id of the motor
/// \param torque The torque value to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_spec_torque(VMC_UCHAR_8 id, VMC_UINT_16 torque) {
    if (id >= 0 && id < _NUM_MOTORS_ && torque > 0 && torque <= _SPEC_TORQUE_) {
	    motor[id].hardware.spec_Torque = torque;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the direction of the motor
/// \param id The id of the motor
/// \param direction the direction to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_direction(VMC_UCHAR_8 id, VMC_UINT_16 direction) {
    if (id >= 0 && id < _NUM_MOTORS_ && (direction == 0  || direction == 1)) {
	     motor[id].hardware.direction = direction;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the command timeout of the controller in cycles
/// \param id The id of the motor
/// \param timeout The timeout value to set
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_timeout(VMC_UINT_16 timeout) {
    if (timeout > 0 && timeout <= _TIMEOUT_) {
	    vmcconfig.timeout = timeout;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the flag if the controller should be used to control RPMs
/// \param flag 0: not active 1: active
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_controller_active(VMC_UCHAR_8 id, VMC_UCHAR_8 flag) {
    if (id >= 0 && id < _NUM_MOTORS_ && (flag == 0  || flag == 1)) {
	    motor[id].controller.controller_active = flag;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the flag if the controller should be used to control RPMs
/// \param flag 0: not active 1: active
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_currentlimiter_active(VMC_UCHAR_8 id, VMC_UCHAR_8 flag) {
    if (id >= 0 && id < _NUM_MOTORS_ && (flag == 0  || flag == 1)) {
	    motor[id].controller.current_limiter_active = flag;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Sets the flag if the Inputs are PWMs or RPMs for the direct mode
/// \param flag 0: RPM 1: PWM
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 set_use_PWM(VMC_UCHAR_8 id, VMC_UCHAR_8 flag) {
    if (id >= 0 && id < _NUM_MOTORS_ && (flag == 0  || flag == 1)) {
	    motor[id].controller.use_PWM = flag;
		return 1;
	}
	else {
		return 0;
	}
}
//////////////////////////////////////////////////////////////////////////////

//============================ set functions end ====================================



//@}
