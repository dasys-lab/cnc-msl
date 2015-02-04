//////////////////////////////////////////////////////////////////////////////
/// \ingroup configdata
//@{
/// \file ais_configdata.h
///
/// \brief Header file of the configuration data module
///
/// \author Pascal Langenberg
///
/// \version 0.1
///
/// \date 12.05.2006
///
//////////////////////////////////////////////////////////////////////////////


#ifndef AIS_MOTORCONFIG_H
#define AIS_MOTORCONFIG_H



// ---------------------------------

// ---------------------------------
// Parameter Ranges
#define _MAX_CUR_		8000
#define _MAX_RPM_		25000
#define _NOM_CUR_		8000
#define _NOM_RPM_		25000
#define _GEAR_RATIO_	10000
#define _WHEEL_RADIUS_	1000
#define _AXE_LENGTH_	1000
#define _TICKS_ROT_		5000
#define _SPEC_TORQUE_	1000
#define _MAX_TEMP_		200
#define _NOM_TEMP_		200
#define _ENV_TEMP_		100
#define _WINDING_T_		500
#define _WINDING_GN_	100
#define _WINDING_GZ_	1000
#define _CHASSIS_T_		500
#define _CHASSIS_GN_	100
#define _CHASSIS_GZ_	1000
#define _CONFIG_		4
#define _TIMEOUT_		1000

// ---------------------------------


#include "ais_typedef.h"

/// Structure for Temperature model parameter and runtime values
struct TEMP {
    /// max Temp in Celsius[0...200]
    VMC_UINT_16 max_Temp;
	/// nominal Temp in Celsius[0...200]
    VMC_UINT_16 nom_Temp;
	/// environment temperature in Celsius[0...100]
    VMC_UINT_16 env_Temp;
    /// time constant for the winding in ms[0...500]
    VMC_UINT_16 winding_t;
	/// denominator of the thermic resistor of the winding in 1/KW [1...100]
    VMC_UINT_16 winding_g_n;
	/// nominator of the thermic resistor of the winding in 1/KW [1...1000]
    VMC_UINT_16 winding_g_z;
	/// time constant for the chassis in ms[0...500]
    VMC_UINT_16 chassis_t;
	/// denominator of the thermic resistor of the chassis in 1/KW [1...100]
    VMC_UINT_16 chassis_g_n;
    /// nominator of the thermic resistor of the chassis in 1/KW [1...1000]
    VMC_UINT_16 chassis_g_z;
};

/// Structure for Hardware properties of the robot
struct HARDWARE {
    /// gear reduction in rots motor/wheel * 10 [1...10000]
    VMC_UINT_16 gear_Reduction;
	/// wheel radius in mm[1...1000]
    VMC_UINT_16 wheel_Radius;
	/// encoder ticks per rot[1...5000]
    VMC_UINT_16 ticks_Rot;
	/// specific Torque in mNm/A [1...1000]
	VMC_UINT_16 spec_Torque;
	/// direction of the motor, 0: normal 1:not normal
	VMC_UINT_16 direction;
};

/// Structure for Configuration of the Controller
struct CONTROLLER {
    /// flag if the controller should be used
    VMC_UCHAR_8 controller_active;
	/// flag if the controller should use RPM or PWM
    VMC_UCHAR_8 use_PWM;
	/// flag if Current Limiter is active
	VMC_UCHAR_8 current_limiter_active;

};

/// Structure for Configuration of a Motor
typedef struct MOTORCONFIG {
    /// maximum current in mA[0...8000]
    VMC_UINT_16 max_Current;
	/// maximum RPM in RPM[0...25000]
    VMC_UINT_16 max_RPM;
	/// nominal current in mA[0...8000]
    VMC_UINT_16 nom_Current;
	/// nominal RPM in RPM[0...25000]
    VMC_UINT_16 nom_RPM;
	/// Structure contains values for thermic model
    struct TEMP temp;
	/// Structure contains hardware properties
    struct HARDWARE hardware;
	/// Structure for Configuration of the Controller
	struct CONTROLLER controller;
};

/// Structure for the motor independent configuration parameter
typedef struct VMCCONFIG {
	/// Communictaion timeout in ms
	VMC_UINT_16 timeout;
};

void set_default_motorconfig();

VMC_INT_16 convert_RPM2PWM(VMC_INT_16 rpm, VMC_INT_16 max_rpm, VMC_INT_16 pwm_period );
VMC_INT_16 convert_PWM2RPM(VMC_INT_16 pwm, VMC_INT_16 max_rpm, VMC_INT_16 pwm_period );

// Get functions

struct TEMP *get_temp_params(VMC_UCHAR_8 id);
VMC_UINT_16 get_max_current(VMC_UCHAR_8 id);
VMC_UINT_16 get_max_RPM(VMC_UCHAR_8 id);
VMC_UINT_16 get_nom_current(VMC_UCHAR_8 id);
VMC_UINT_16 get_nom_RPM(VMC_UCHAR_8 id);
VMC_UINT_16 get_max_temp(VMC_UCHAR_8 id);
VMC_UINT_16 get_nom_temp(VMC_UCHAR_8 id);
VMC_UINT_16 get_env_temp(VMC_UCHAR_8 id);
VMC_UINT_16 get_winding_t(VMC_UCHAR_8 id);
VMC_UINT_16 get_winding_g_z(VMC_UCHAR_8 id);
VMC_UINT_16 get_winding_g_n(VMC_UCHAR_8 id);
VMC_UINT_16 get_chassis_t(VMC_UCHAR_8 id);
VMC_UINT_16 get_chassis_g_z(VMC_UCHAR_8 id);
VMC_UINT_16 get_chassis_g_n(VMC_UCHAR_8 id);
VMC_UINT_16 get_gear_reduction(VMC_UCHAR_8 id);
VMC_UINT_16 get_wheel_radius(VMC_UCHAR_8 id);
VMC_UINT_16 get_ticks_rot(VMC_UCHAR_8 id);
VMC_UINT_16 get_spec_torque(VMC_UCHAR_8 id);
VMC_UINT_16 get_direction(VMC_UCHAR_8 id);
VMC_UCHAR_8 get_controller_active(VMC_UCHAR_8 id);
VMC_UCHAR_8 get_currentlimiter_active(VMC_UCHAR_8 id);
VMC_UCHAR_8 get_use_PWM(VMC_UCHAR_8 id);
VMC_UINT_16 get_timeout();


// Set functions
VMC_UCHAR_8 set_max_current(VMC_UCHAR_8 id, VMC_UINT_16 current);
VMC_UCHAR_8 set_max_RPM(VMC_UCHAR_8 id, VMC_UINT_16 rpm);
VMC_UCHAR_8 set_nom_current(VMC_UCHAR_8 id, VMC_UINT_16 current);
VMC_UCHAR_8 set_nom_RPM(VMC_UCHAR_8 id, VMC_UINT_16 rpm);
VMC_UCHAR_8 set_max_temp(VMC_UCHAR_8 id, VMC_UINT_16 temp);
VMC_UCHAR_8 set_nom_temp(VMC_UCHAR_8 id, VMC_UINT_16 temp);
VMC_UCHAR_8 set_env_temp(VMC_UCHAR_8 id, VMC_UINT_16 temp);
VMC_UCHAR_8 set_winding_t(VMC_UCHAR_8 id, VMC_UINT_16 t);
VMC_UCHAR_8 set_winding_g_z(VMC_UCHAR_8 id, VMC_UINT_16 g_z);
VMC_UCHAR_8 set_winding_g_n(VMC_UCHAR_8 id, VMC_UINT_16 g_n);
VMC_UCHAR_8 set_chassis_t(VMC_UCHAR_8 id, VMC_UINT_16 t);
VMC_UCHAR_8 set_chassis_g_z(VMC_UCHAR_8 id, VMC_UINT_16 g_z);
VMC_UCHAR_8 set_chassis_g_n(VMC_UCHAR_8 id, VMC_UINT_16 g_n);
VMC_UCHAR_8 set_gear_reduction(VMC_UCHAR_8 id, VMC_UINT_16 reduction);
VMC_UCHAR_8 set_wheel_radius(VMC_UCHAR_8 id, VMC_UINT_16 radius);
VMC_UCHAR_8 set_axe_length(VMC_UCHAR_8 id, VMC_UINT_16 length);
VMC_UCHAR_8 set_ticks_rot(VMC_UCHAR_8 id, VMC_UINT_16 ticks);
VMC_UCHAR_8 set_spec_torque(VMC_UCHAR_8 id, VMC_UINT_16 torque);
VMC_UCHAR_8 set_direction(VMC_UCHAR_8 id, VMC_UINT_16 direction);
VMC_UCHAR_8 set_controller_active(VMC_UCHAR_8 id, VMC_UCHAR_8 state);
VMC_UCHAR_8 set_use_PWM(VMC_UCHAR_8 id, VMC_UCHAR_8 state);
VMC_UCHAR_8 set_currentlimiter_active(VMC_UCHAR_8 id, VMC_UCHAR_8 state);
VMC_UCHAR_8 set_timeout(VMC_UINT_16);

TMC_ULONG_32 clear_abs_Rots(VMC_UCHAR_8 id);		//@@@@@

#endif /* AIS_MOTORCONFIG_H */

//@}