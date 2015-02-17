//////////////////////////////////////////////////////////////////////////////
/// \defgroup motorcmd Command Interpreter for Motor control on MPWR Board
//@{
/// \file ais_motorcmd.c
///
/// \brief 	All functions needed for Motorcommand-Interpretation
///
/// This module contains all functions for the analyzing of incoming and
/// building of outgoing commands to the communication modul\n
/// To use this modul with incoming commands you have to call the functions
//  ais_mctrl_config(), ais_mctrl_motorcontrol(), ais_mctrl_status1() or
/// ais_mctrl_status2() with a command as command struct as parameter to handle
/// this command, dependent on which command group the command belongs to\n
///
/// To use this modul to build outgoing commands to have to call the functions
/// make_status_command(),which returns a command which can be send back.\n
///
/// \note When you add a parameter to the config of the TMC 2000 you have to
/// do the following steps to integrate it into this module:\n
/// 1. Add a macro with a definition of the command# for the config of this
///    parameter into the ais_motorcmd.h\n
/// 2. Add a macro with a definition of the range of the parameter\n
/// 3. To the function ais_mctrl_config() add the cases to the switch
///    expressions, use existing as templates\n
/// \n
/// When you add a variable to the runtime data of the TMC 2000 you have to the
/// following steps to inegrate it into the module: \n
/// 1. In the motorcmd.h, add a macro for the id of the status request for
///    the parameter
/// 2. To the function get_Parameter, add a case with get function for the
///    parameter\n
///
/// \author Adam Cwientzek and Pascal Langenberg
///
/// \version 0.9
///
/// \date 1.08.2006
///
//////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <reg167.h>
#include <INTRINS.H>
#include <stdlib.h>
#include <intrins.h>
#include <string.h>

#include "ais_typedef.h"
#include "can_stuff/ais_can.h"
#include "motorctrl/ais_motorcmd.h"

#include "system/ais_astring.h"
#include "system/ais_utils.h"
#include "system/ais_cmdbuff.h"
#include "system/ais_select.h"
#include "aisc167b/ais_bioport.h"
#include "can_stuff/ais_can.h"

#include "motorctrl/ais_configmanager.h"
#include "motorctrl/ais_motorconfig.h"
#include "motorctrl/ais_motorctrl.h"

#include "controller/ais_controller.h"

#include "aisc167b/ais_adc.h"
#include "aisc167b/ais_gpt.h"

// declaration of one motormotorstate structure per motor
extern struct MOTORSTATE motorstate[_NUM_MOTORS_];


//////////////////////////////////////////////////////////////////////////////
/// \brief returns a runtime Parameter to a given status request
//////////////////////////////////////////////////////////////////////////////
TMC_ULONG_32 get_Parameter(VMC_UCHAR_8 id, VMC_UCHAR_8 cmd) {
// Function could be macro ?
    switch ( cmd ) {
        case _SREG_MOT_RPM_ 		: return get_act_RPM(id);
        case _SREG_MOT_PWM_ 		: return get_set_PWM(id);
		case _SREG_MOT_VOLTAGE_ 	: return get_act_Voltage(id);
		case _SREG_MOT_CURRENT_ 	: return get_act_Current(id);
		case _SREG_MOT_TEMP_		: return get_act_Temp(id);
		case _SREG_MOT_TORQUE_		: return get_act_Torque(id);
    	case _SREG_MOT_POWER_		: return get_act_Power(id);
	   	case _SREG_MOT_TICKS_ABS_ 	: return get_abs_Ticks(id);
	    case _SREG_MOT_TICKS_REL_ 	: return get_rel_Ticks(id);
		case _SREG_BATTERY_ 		: return get_batteryvoltage();
		case _SREG_Digital_Input_	: return get_dioport();
		case _SREG_Analog1_Input_ 	: return get_a1_ioport();
		case _SREG_Analog2_Input_ 	: return get_a2_ioport();
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Send a range error to the Command Buffer, quick and dirty...
//////////////////////////////////////////////////////////////////////////////
void ERR_range(struct CmdStruct *ErrResponse, char cmdGrp) {
	ErrResponse->cmdgrp  = cmdGrp;
    ErrResponse->cmd     = _RANGE_ERR_;
	ErrResponse->datalen = 0;
    cmdb_dataAppendStr( ErrResponse,"Parameter out of range");
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Send a unkown error to the Command Buffer, quick and dirty...
//////////////////////////////////////////////////////////////////////////////
void ERR_unknown(struct CmdStruct *ErrResponse, char cmdGrp) {
	ErrResponse->cmdgrp  = cmdGrp;
    ErrResponse->cmd     = _UNKNOWN_ERR_;
	ErrResponse->datalen = 0;
	cmdb_dataAppendStr(ErrResponse,"Err");
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Send a param error to the Command Buffer, quick and dirty...
//////////////////////////////////////////////////////////////////////////////
void ERR_param(struct CmdStruct *ErrResponse, char cmdGrp) {
	ErrResponse->cmdgrp  = cmdGrp;
    ErrResponse->cmd     = _PARAM_ERR_;
	ErrResponse->datalen = 0;
	cmdb_dataAppendStr(ErrResponse,"Err Param");
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief build a command struct for a status response
/// This function builds a command for the status1 group, the normal group for
/// status responses as answer to an status reqest and returns it\n
/// As parameters it gets a channel id and the command id of the status request\n
/// If the channel id is 0, the response will be build for any motor, if not
/// for the special channel id\n
/// The function is called by the funtion send_Status() of the modul ais_status,
/// which itself is called periodic by the control loop
///
/// \param channelID the motor for which the response is requested, 0 = all
/// \param asrequest the command id of the status request command ?
//////////////////////////////////////////////////////////////////////////////
void make_status_command(struct CmdStruct *ResponseCmd, VMC_UCHAR_8 channelID, VMC_UCHAR_8 requestID, VMC_UCHAR_8 *asrequest) {

	VMC_UCHAR_8 motor, reqPos;		// Loop counter
	VMC_UCHAR_8 startLoop;			// Loop Parameters

	// Init Response Frame
	ResponseCmd->cmdgrp  	= _CMDGRP_MOTOR_STATUSOUT_; // Motor Response
	ResponseCmd->cmd 	 	= requestID;	 // Init Command is request ID
	ResponseCmd->data[0] 	= channelID;	 //	Save Channel ID
	ResponseCmd->datalen 	= 1;			 // One Byte Information in Datalen

	// Check if Requests are for one specified Channel only
    if ( channelID > 0 ) {
	   startLoop = _NumToIndex_(channelID); // Yes, so start with this Channel (Index = Channel - 1 !!)
	} else {
		startLoop = 0;
	}    // No, start from Index 0 and go through all Channels


    if ( !_IS_MOTORID_(channelID) ) { // Range Error: Motor ID !!!! channel(0)==ALL/NumMotor==1|2|3
		ERR_range(ResponseCmd, _CMDGRP_MOTOR_ERR_);
		cmdb_set_command( ResponseCmd );
		return;
	}


	//printf("startLoop %d ", startLoop);

	// No Process the Requests
	for (reqPos = 1; reqPos <= astring_len(asrequest); reqPos++) {
 	 for (motor = startLoop; motor < _NUM_MOTORS_; motor++) {
       switch ( asrequest[reqPos] ) {
	     // Requests with LONG output
	     case _SREG_MOT_TICKS_ABS_:
		 	cmdb_dataAppendLong( ResponseCmd, get_Parameter(motor, asrequest[reqPos]));
			break;
		 // Requests with INT output
		 default				 :
		 	cmdb_dataAppendInt( ResponseCmd, (VMC_UINT_16)get_Parameter(motor, asrequest[reqPos]));
			break;
	   } // switch

//@@@@@@@@@@@@@@@
// append timestamp get_cycle_time_part();

cmdb_dataAppendLong( ResponseCmd, get_cycle_time_part());

	  if ( channelID > 0 ) break; // Only one Channel was requested, Break channel loop
	 } // for (Channel loop)
	} // for (Request processing loop)
}
//////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////////
/// \brief Analyze config commands and change configuration
/// This function gets a command with config parameter as data and change the
/// configuration if applicable. It calls setter functions of the modul
/// ais_confifdata. It is called by the the function sel_proceed_cmd() of the
/// module ais_select, which itself is called periodic by the main loop
/// \param Config command struct
//////////////////////////////////////////////////////////////////////////////
void ais_mctrl_config(struct CmdStruct Command) {

	//============================Declaration of local variables==============

	VMC_UCHAR_8 cmd 	 		= Command.cmd;	// char for command type
	VMC_UCHAR_8 chkParam 		= 1;			// Parameter within Range, Default : True

	// Variable for the channel to config, (Range 1 to 3) is converted to index (Range 0 to 2)
	VMC_UCHAR_8 channelID 		= Command.data[0] - 1;    
	VMC_UINT_16 param[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};	// array for Integer parameter in command

	// Response to Host (Default parameter range error)
	struct CmdStruct ResponseCmd;
	ERR_range(&ResponseCmd, _CMDGRP_MOTOR_ERR_ );
//can_debug(0xbc,2);

	//============================Declaration of local variables end==========
	// Check if selected Channel is within Range
	if (Command.data[0] == 0 || Command.data[0] > _NUM_MOTORS_) {
		cmdb_set_command( &ResponseCmd );
		return;
	}
//can_debug(0xcd,2);

	//============================Extract parameter from command==============
	// at the moment in maximum just one parameter

	if (Command.datalen > 1) {
		param[0] = _DATA_TO_INT_(Command.data[1], Command.data[2]);
	}
	//============================Extract parameters from command end=========

	//============================Change configuration========================
	// analyze type of command and change the configuration if applicable

	// if setter is called and Parameter is out of range, Flag chkParam will be cleared !!

	switch ( cmd ) {

		case _MCMD_CONTROLLER_:
			if (Command.datalen > 1) // Parameters available, call setter
				chkParam = set_controller_active(channelID, param[0]);
			if ( chkParam ) { // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_controller_active(channelID) ); // Append Data
			}
			break;

		case _MCMD_USE_PWM_:
			if (Command.datalen > 1) // Parameters available, call setter
		 		chkParam = set_use_PWM(channelID, param[0]);
			if ( chkParam ) {  // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_use_PWM(channelID) ); // Append Data
			}
			break;

		case _MCMD_LIMITER_:
			if (Command.datalen > 1) // Parameters available, call setter
				chkParam = set_currentlimiter_active(channelID, param[0]);
			if ( chkParam ) { // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_currentlimiter_active(channelID) ); // Append Data
			}
			break;

		case _MCMD_TIMEOUT_:

			if (Command.datalen > 0) // Parameters available, call setter ( NO CHANNEL ID FOR TIMEOUT !!!! )
				 chkParam = set_timeout(_DATA_TO_INT_(Command.data[0], Command.data[1]));
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_timeout() ); // Append Data
			}
			break;

		case _MCMD_MAX_CUR_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_max_current(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_max_current(channelID) ); // Append Data
			}
			break;

		case _MCMD_NOM_CUR_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_nom_current(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_nom_current(channelID) ); // Append Data
			}
			break;

		case _MCMD_MAX_RPM_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_max_RPM(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_max_RPM(channelID) ); // Append Data
			}
			break;

		case _MCMD_NOM_RPM_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_nom_RPM(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_nom_RPM(channelID) ); // Append Data
			}
			break;

		case _MCMD_GEAR_RATIO_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_gear_reduction(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_gear_reduction(channelID) ); // Append Data
			}
			break;

		case _MCMD_WHEEL_RADIUS_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_wheel_radius(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_wheel_radius(channelID) ); // Append Data
			}
			break;

		case _MCMD_TICKS_ROT_ :
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_ticks_rot(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_ticks_rot(channelID) ); // Append Data
			}
			break;

		case _MCMD_SPEC_TORQUE_	 :
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_spec_torque(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_spec_torque(channelID) ); // Append Data
			}
			break;

 		case _MCMD_DIRECTION_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_direction(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_direction(channelID) ); // Append Data
			}
			break;

		case _MCMD_MAX_TEMP_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_max_temp(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_max_temp(channelID) ); // Append Data
			}
			break;

		case _MCMD_NOM_TEMP_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_nom_temp(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_nom_temp(channelID) ); // Append Data
			}
			break;

		case _MCMD_ENV_TEMP_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_env_temp(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_env_temp(channelID) ); // Append Data
			}
			break;

		case _MCMD_WINDING_T_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_winding_t(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_winding_t(channelID) ); // Append Data
			}
			break;

		case _MCMD_WINDING_GN_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_winding_g_n(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_winding_g_n(channelID) ); // Append Data
			}
			break;

		case _MCMD_WINDING_GZ_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_winding_g_z(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_winding_g_z(channelID) ); // Append Data
			}
			break;

		case _MCMD_CHASSIS_T_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_chassis_t(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_chassis_t(channelID) ); // Append Data
			}
			break;

		case _MCMD_CHASSIS_GN_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_chassis_g_n(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_chassis_g_n(channelID) ); // Append Data
			}
			break;

		case _MCMD_CHASSIS_GZ_:
			if (Command.datalen > 1) // Parameters available, call setter
				 chkParam = set_chassis_g_z(channelID, param[0]);
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, get_chassis_g_z(channelID) ); // Append Data
			}
			break;

		case _MCMD_SAVE_CONFIG_ :
		    // Parameters available and ID within range
			if ((Command.datalen > 1) && (channelID < _CONFIG_) && (channelID >= 0) ) 
			     chkParam = save_configuration(param[0]-1); // Ignore channel, use parameter as configuration id
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				// We do not have Channel info in save/load commands !!
				ResponseCmd.data[0] = 0; // so insert just 0-Value, just to keep Format as all other Responses
				cmdb_dataAppendInt( &ResponseCmd, param[0] ); // Return memory slot
			}
			break;

		case _MCMD_LOAD_CONFIG_ :
		    // Parameters available and ID within range
			if ((Command.datalen > 1) && (channelID < _CONFIG_) && (channelID >= 0) ) 
			     chkParam = load_configuration(param[0]-1); // Ignore channel, use parameter as configuration id
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				// We do not have Channel info in save/load commands !!
				ResponseCmd.data[0] = 0; // so insert just 0-Value, just to keep Format as all other Responses
				cmdb_dataAppendInt( &ResponseCmd, param[0] ); // Return memory slot
			}
			break;

		case _MCMD_CONFIG_IOPORT_:
			if (Command.datalen > 1) {// Parameters available, call setter
			

				 chkParam = configurate_ioport( param[0] );
				 }
			if ( chkParam ) { 			 // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = 1; // Return also channel !

				cmdb_dataAppendInt( &ResponseCmd, get_configurate_ioport() ); // Append Data

//cmdb_dataAppendChar(&ResponseCmd, 0x41);
//can_debug(ResponseCmd.datalen,2);
			}
			break;

//=333===================================== WORKING AREA ======================================================

/*		// Clear all Rotation counter ******************************************************
		case _CLEAR_ALL_TICKS_ABS_:
			// call function to clear the ROTs
if (can_send( MSG_CMDGRP_MOTOR_CONFOUT_ )) while(1){led_swap_red();};
			Clear_all_Rots();
			// if optional status Request byte is set, get Status Message
 		    if (Command.datalen == 7)
				// Make short Status Request for all Channels (id=0) and Request Byte
				ais_mctrl_shortStatusRequest(0, Command.data[1]);
	     	break;
*/

		// Clear Rotation counter of one given motor ****************************************
		case _CLEAR_ONE_TICKS_ABS_:
			// call function to set the RPM, - 1 because of external representation of motors begins with 1
			Clear_ref_Rots(_NumToIndex_(Command.data[0]));
			// if optional status Request byte is set, get Status Message
 		    if (Command.datalen == 7)
				// Make short Status Request from ChannelID and Request Byte
				ais_mctrl_shortStatusRequest( _NumToIndex_(Command.data[0]), Command.data[1]);
	     	break;

//===========================================***********=======================================================


		default:
			ERR_unknown(&ResponseCmd, _CMDGRP_MOTOR_ERR_); // Send unknown Error Message !

    } // switch
	//============================Change configuration end====================


	//============================Build and send success response=============

    if ( chkParam ) { // No error occurred, prepare usual response
		ResponseCmd.cmdgrp = _CMDGRP_MOTOR_CONFOUT_; // Grp is "motor config"
		ResponseCmd.cmd    = cmd;    // Same as Request Command
		// Values allready sotred during switch Block, One Int added
		ResponseCmd.datalen = 3;
	}

	// Send command to command buffer
	cmdb_set_command( &ResponseCmd );

	//============================Build and send success response end=========
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Make short status request
//////////////////////////////////////////////////////////////////////////////
void ais_mctrl_shortStatusRequest(VMC_UCHAR_8 channelId, VMC_UCHAR_8 requestID) { // MARK001
 VMC_UCHAR_8 asrequest[5];
 struct CmdStruct ResponseCmd;

 astring_fromChar(asrequest, requestID); // Get Status Request Byte
 make_status_command(&ResponseCmd, channelId, requestID, asrequest); // Make Status Request
 cmdb_set_command(&ResponseCmd); // Send Status
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Analyze motor control commands and change configuration
/// \Command Config command struct
//////////////////////////////////////////////////////////////////////////////
void ais_mctrl_motorcontrol(struct CmdStruct Command) {

	//============================Analyze Command and set values==============
    switch ( Command.cmd ) {

		// Set all PWM Channels **********************************************
    	case _SET_ALL_PWM_:
			set_all_PWM(
			   _DATA_TO_INT_(Command.data[0], Command.data[1]), // PWM Motor 1
			   _DATA_TO_INT_(Command.data[2], Command.data[3]), // PWM Motor 2
			   _DATA_TO_INT_(Command.data[4], Command.data[5])  // PWM Motor 3
			);

			// if optional status Request byte is set, get Status Message
 		    if (Command.datalen == 7)
				// Make short Status Request for all Channels (id=0) and Request Byte
				ais_mctrl_shortStatusRequest(0, Command.data[6]);
	     	break;

		// Set one PWM Channels **********************************************
		case _SET_ONE_PWM_:
			// call function to set the PWM, - 1 because of external representation of motors begins with 1
			set_set_PWM(_NumToIndex_(Command.data[0]), _DATA_TO_INT_(Command.data[1], Command.data[2]));
			// if optional status Request byte is set, get Status Message
 		    if (Command.datalen == 7)
				// Make short Status Request from ChannelID and Request Byte
				ais_mctrl_shortStatusRequest(_NumToIndex_(Command.data[0]),Command.data[6]);
	     	break;


		// Set all RPM ******************************************************
		case _SET_ALL_RPM_:
			// call function to set the PWMs
			set_all_RPM(
			   _DATA_TO_INT_(Command.data[0], Command.data[1]), // RPM Motor 1
			   _DATA_TO_INT_(Command.data[2], Command.data[3]), // RPM Motor 2
			   _DATA_TO_INT_(Command.data[4], Command.data[5])  // RPM Motor 3
			);
			// if optional status Request byte is set, get Status Message
			if (Command.datalen == 7)
				// Make short Status Request for all Channels (id=0) and Request Byte
				ais_mctrl_shortStatusRequest(0, Command.data[6]);
	     	break;


		// Set RPM of one given motor ****************************************
		case _SET_ONE_RPM_:
			// call function to set the RPM, - 1 because of external representation of motors begins with 1
			set_ref_RPM(_NumToIndex_(Command.data[0]), _DATA_TO_INT_(Command.data[1], Command.data[2]));
			// if optional status Request byte is set, get Status Message
 		    if (Command.datalen == 7)
				// Make short Status Request from ChannelID and Request Byte
				ais_mctrl_shortStatusRequest( _NumToIndex_(Command.data[0]), Command.data[6]);
	     	break;


	   		// Set all OUTPUT ******************************************************
		case _SET_DIGITAL_IO_:  												// 0x52!!
			// call function to set the DigitalIO

			set_dioport(
					   _DATA_TO_INT_(Command.data[0], Command.data[1])
			);

			// if optional status Request byte is set, get Status Message
 		    if (Command.datalen == 7)    //7
				// Make short Status Request for all Channels (id=0) and Request Byte
				ais_mctrl_shortStatusRequest(0, Command.data[6]);
	     	break;
			

   	   		// clear all AbsolutTicks ******************************************************
		case _SET_ALL_TICKS_ABS_CLEAR_:  												// 0x52!!
			// call function to clear all ticks

			Clear_all_Rots();

			// if optional status Request byte is set, get Status Message
 		    if (Command.datalen == 7)    //7
				// Make short Status Request for all Channels (id=0) and Request Byte
				ais_mctrl_shortStatusRequest(0, Command.data[6]);
	     	break;


  	   		// set bumper message ON ******************************************************
		case _SET_MODE_WITH_BUMPER_:  												// 0x52!!
			// call function to clear all ticks

			Switch_bumper_mode();

			// if optional status Request byte is set, get Status Message
 		    if (Command.datalen == 7)    //7
				// Make short Status Request for all Channels (id=0) and Request Byte
				ais_mctrl_shortStatusRequest(0, Command.data[6]);
	     	break;
			


//===========================================***********=======================================================
    }

	//============================Analyze Command and set values end==========

}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Set status byte to status request and its channel to the channel
/// of the request command
/// \Command Config command struct
//////////////////////////////////////////////////////////////////////////////
void ais_mctrl_status1(struct CmdStruct Command){

	VMC_UCHAR_8 build_string[255];
	VMC_UCHAR_8 ChannelID;
	VMC_UCHAR_8 requestID = Command.cmd;

	// if self-builded response is reqested
    if (requestID == _SREG_USER_REQUEST_) {
	    astring_create(build_string, Command.datalen, Command.data);
		// first parameter is channel
	    ChannelID = build_string[1];
	} else { 	// if just one parameter is requested
	    astring_fromChar(build_string, requestID);
		// If Data is appended, then it is a single channel request
		if ( Command.datalen ) {
				ChannelID = Command.data[0];
		} else { // otherwise it is a request for all channels
				ChannelID = 0;

		} // If Datelen
	} // if requestID

	// build response
	make_status_command(&Command, ChannelID, requestID, build_string);
	Command.cmd = requestID;
	cmdb_set_command(&Command);
}

//===================================== Working area ============================
//
//////////////////////////////////////////////////////////////////////////////
/// \brief Set RPM for three motors equal zero
/// This function calls the setter for the set RPMs of the modul ais_control
 
/////////////////////////////////////////////////////////////////////////////
void Clear_all_Rots(void) {
	Clear_ref_Rots(0);
	Clear_ref_Rots(1);
	Clear_ref_Rots(2);
}

/////////////////////////////////////////////////////////////////////////////
//=================================================================================

//////////////////////////////////////////////////////////////////////////////
/// \brief Clears ALL rotation counter of the motor 		@@@@@
/// \param id The id of the motor
/// \param rpm The RPM to set
//////////////////////////////////////////////////////////////////////////////
void Clear_ref_Rots(VMC_UCHAR_8 id) {
	    motorstate[id].ticks.abs_Ticks = 0;
}
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
/// \brief switch the VMC to bumpermode
/// \param none
/// \param rpm The RPM to set
//////////////////////////////////////////////////////////////////////////////

void Switch_bumper_mode(void){
			mode_flag	= 0;	// port M3 IO pins ONLY for bumper - action
}
//////////////////////////////////////////////////////////////////////////////
//@}


