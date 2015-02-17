//////////////////////////////////////////////////////////////////////////////
/// \defgroup configsave Saving/Loading of the Controller configuration to eeprom
//@{
/// \file ais_configsave.c
///
/// \brief 	Functions for saving and loading the configuration
///
/// This modul contains functions to save and load the configuration of the TMC 2000 \n
/// to the eeprom which is connected the I²C Bus. It uses functions of the modul ais_eeprom, which contains functions \n
/// uses functions of the modul ais_i2c to store and load Data via the I²C Bus.\n
/// In the ais_configsave.h file there are the definitions of base addresses of 5 (extendable) possible\n
/// "Saving areas", the first (0) is used for the default config, the other are free usable.\n
///
///
/// \note To add a new Parameter to this modul you have to do the following steps:\n
/// 1. Add a macro with the address offset of the parameter to the ais_configsave.h, take care if motor independent or not\n
/// 2. Function save_configuration:\n
///  Add a call of the function i2c_storeInt() in the chapter motor independent parameter, if parameter is motor independent, use other calls as template\n
///  Add a call of the function i2c_storeInt() in the chapter motor  parameter, if parameter is motor dependent, use other calls as template\n
/// 3. Function load_configuration:\n
///  Add a call of the function i2c_loadInt() in the chapter motor independent parameter, if parameter is motor independent, use other calls as template\n
///  Add a call of the fucntion i2c_loadInt() in the chapter motor  parameter, if parameter is motor dependent, use other calls as template\n
///
/// \author Pascal Langenberg
///
/// \version 0.2
///
/// \date 09.08.2006
///


//////////////////////////////////////////////////////////////////////////////

#include "ais_typedef.h"
#include "aisc167b/ais_i2c.h"
#include "aisc167b/ais_eeprom.h"
#include "motorctrl/ais_configmanager.h"
#include "motorctrl/ais_motorconfig.h"
#include "controller/ais_controller.h"
#include "aisc167b/ais_gpt.h"

//#include "control/ais_motorctrl.h"
#include "comasc/ais_asc0.h"
//
#include <stdio.h>

void set_default_configuration() {

  // default motor configuration (in ais_motorconfig)
  set_default_motorconfig();

  // default controller configuration (in ais_controller)
  default_controller_parameters();
}


//////////////////////////////////////////////////////////////////////////////
/// \brief Saves the complete current configuration parameters to the eeprom
///  When this function is called, it marks the given memory area with a flag and writes the current config parameters to the specified addresses (.h)
/// \param config The Memory Area where the configuration should be saved [1...4]
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 save_configuration(VMC_UCHAR_8 config ) {

	
	//loop counter
	VMC_UCHAR_8 i;

	//variables to save the calculated memory addresses
	VMC_UINT_16 base;
	VMC_UINT_16 motor;



	// Which memory area is the target-->which address is the actual base address
	base = (_OFFSET_CONFIG_ * config) + _OFFSET_BASE_;

	i2c_init();
	//Set the flag
	i2c_storeByte(0xaa, base + _ADDR_FLAG_);


	//Write the motor independent Parameter to the eeprom, addresses in headerfile

	i2c_storeInt(get_timeout(), base + (2* _ADDR_TIMEOUT_) );

	//For every motor: calculate a motorspecific offset
	for(i = 0; i < _NUM_MOTORS_; i++) {

		motor = base + ((i + 1) * _OFFSET_MOTOR_ );
		//and write all motorrdepending parameters to the eeprom
		i2c_storeInt(get_max_current(i), 	motor + (2 * _ADDR_MAX_CURRENT_) );
		i2c_storeInt(get_max_RPM(i), 		motor + (2 * _ADDR_MAX_RPM_) );
		i2c_storeInt(get_nom_current(i), 	motor + (2 * _ADDR_NOM_CURRENT_) );
		i2c_storeInt(get_nom_RPM(i), 		motor + (2 * _ADDR_NOM_RPM_) );
		i2c_storeInt(get_gear_reduction(i), motor + (2 * _ADDR_GEAR_REDUCTION_) );
		i2c_storeInt(get_wheel_radius(i), 	motor + (2 * _ADDR_WHEEL_RADIUS_) );

		i2c_storeInt(get_spec_torque(i), 	motor + (2 * _ADDR_SPEC_TORQUE_) );
		i2c_storeInt(get_ticks_rot(i), 		motor + (2 * _ADDR_TICKS_ROT_) );
		i2c_storeInt(get_max_temp(i), 		motor + (2 * _ADDR_MAX_TEMP_) );
		i2c_storeInt(get_nom_temp(i), 		motor + (2 * _ADDR_NOM_TEMP_) );
		i2c_storeInt(get_winding_t(i), 		motor + (2 * _ADDR_WINDING_T_) );
		i2c_storeInt(get_winding_g_z(i), 	motor + (2 * _ADDR_WINDING_G_Z_) );
		i2c_storeInt(get_winding_g_n(i), 	motor + (2 * _ADDR_WINDING_G_N_) );
		i2c_storeInt(get_chassis_t(i), 		motor + (2 * _ADDR_CHASSIS_T_) );
		i2c_storeInt(get_chassis_g_z(i), 	motor + (2 * _ADDR_CHASSIS_G_Z_) );	
		i2c_storeInt(get_chassis_g_n(i), 	motor + (2 * _ADDR_CHASSIS_G_N_) );

		// Controller Parameters
		i2c_storeInt(ctrl_get_Kpr(i), 		motor + (2 * _ADDR_KP_) );
		i2c_storeInt(ctrl_get_Tn(i), 		motor + (2 * _ADDR_TN_) );
		i2c_storeInt(ctrl_get_Tv(i), 		motor + (2 * _ADDR_TV_) );
		i2c_storeInt(ctrl_get_deadband(i), 	motor + (2 * _ADDR_DEADBAND_) );

		i2c_storeInt(ctrl_get_nRamp(i), 	motor + (2 * _ADDR_NRAMP_) );
		i2c_storeInt(ctrl_get_pRamp(i), 	motor + (2 * _ADDR_PRAMP_) );

		i2c_storeInt(get_direction(i), 				motor + (2 * _ADDR_DIRECTION_) );
		i2c_storeInt(get_controller_active(i), 		motor + (2 * _ADDR_CONTROLLER_) );
		i2c_storeInt(get_use_PWM(i), 				motor + (2 * _ADDR_USEPWM_) );
		i2c_storeInt(get_currentlimiter_active(i), 	motor + (2 * _ADDR_LIMITER_) );

	}
	//Initializize the cycletime counter because of the "break"

	init_cycletime_counter();
    return 1;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Loads the complete configuration parameters from the eeprom
/// When this function is called it check the "save flag" and loads the config from the given area
/// If the flag is not set, i.e. there is no conig at this area, the default config will be loaded
/// \param config The Memory Area from which the configuration should be load
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 load_configuration(VMC_UCHAR_8 config ) {

	//loop counter
	VMC_UCHAR_8 i;

	//variables to save the calculated memory addresses
	VMC_UINT_16 base;
	VMC_UINT_16 motor;
	// Which memory area is the target-->which address is the actual base address
	base = (_OFFSET_CONFIG_ * config) + _OFFSET_BASE_;

	//load the configuation only if the flag is set to the specified value 0xaa
	if (i2c_loadByte(base + _ADDR_FLAG_) == 0xaa) {

		//Load the motor independent Parameter from the eeprom, addresses in headerfile

		set_timeout(i2c_loadInt(base + (2 * _ADDR_TIMEOUT_) ));
				
		//For every motor: calculate a motorspecific offset
		for(i = 0; i < _NUM_MOTORS_; i++) {
			motor = base + (i + 1) * _OFFSET_MOTOR_;

				//and write all motorrdepending parameters to the eeprom
			set_max_current(i, 			i2c_loadInt(motor + (2 * _ADDR_MAX_CURRENT_)));
			set_max_RPM(i, 				i2c_loadInt(motor + (2 * _ADDR_MAX_RPM_)));
			set_nom_current(i, 			i2c_loadInt(motor + (2 * _ADDR_NOM_CURRENT_)));
			set_nom_RPM(i,				i2c_loadInt(motor + (2 * _ADDR_NOM_RPM_)));
			set_gear_reduction(i, 		i2c_loadInt(motor + (2 * _ADDR_GEAR_REDUCTION_)));
			set_wheel_radius(i,			i2c_loadInt(motor + (2 * _ADDR_WHEEL_RADIUS_)));

			set_ticks_rot(i,			i2c_loadInt(motor + (2 * _ADDR_TICKS_ROT_)));
			set_spec_torque(i,			i2c_loadInt(motor + (2 * _ADDR_SPEC_TORQUE_)));			
			set_max_temp(i, 			i2c_loadInt(motor + (2 * _ADDR_MAX_TEMP_)));
			set_nom_temp(i, 			i2c_loadInt(motor + (2 * _ADDR_NOM_TEMP_)));
			set_winding_t(i, 			i2c_loadInt(motor + (2 * _ADDR_WINDING_T_)));
			set_winding_g_z(i, 			i2c_loadInt(motor + (2 * _ADDR_WINDING_G_Z_)));
			set_winding_g_n(i, 			i2c_loadInt(motor + (2 * _ADDR_WINDING_G_N_)));
			set_chassis_t(i, 			i2c_loadInt(motor + (2 * _ADDR_CHASSIS_T_)));
			set_chassis_g_z(i,			i2c_loadInt(motor + (2 * _ADDR_CHASSIS_G_Z_)));
			set_chassis_g_n(i,			i2c_loadInt(motor + (2 * _ADDR_CHASSIS_G_N_)));


			// Controller Parameters
			ctrl_set_Kpr(i,				i2c_loadInt(motor + (2 * _ADDR_KP_) ) );
			ctrl_set_Tn(i,				i2c_loadInt(motor + (2 * _ADDR_TN_) ) );
			ctrl_set_Tv(i,				i2c_loadInt(motor + (2 * _ADDR_TV_) ) );
			ctrl_set_deadband(i,		i2c_loadInt(motor + (2 * _ADDR_DEADBAND_) ) );

			ctrl_set_nRamp(i,			i2c_loadInt(motor + (2 * _ADDR_NRAMP_) ) );
			ctrl_set_pRamp(i,			i2c_loadInt(motor + (2 * _ADDR_PRAMP_) ) );

			set_direction(i,			i2c_loadInt(motor + (2 * _ADDR_DIRECTION_)));
			set_controller_active(i, 	i2c_loadInt(motor + _ADDR_CONTROLLER_));
			set_use_PWM(i, 				i2c_loadInt(motor + _ADDR_USEPWM_));
			set_currentlimiter_active(i, i2c_loadInt(motor + _ADDR_LIMITER_));

	    }

	}
    else {
	    set_default_configuration();
		//set_default_Reg_Params();
	}
	//Initializize the cycletime counter because of the "break"
	init_cycletime_counter();
    return 1;
}
//////////////////////////////////////////////////////////////////////////////
//@}
