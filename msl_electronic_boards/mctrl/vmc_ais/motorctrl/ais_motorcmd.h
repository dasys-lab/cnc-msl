//////////////////////////////////////////////////////////////////////////////
/// \defgroup motorcmd Command Interpreter for Motor control on MPWR Board
//@{
/// \file ais_motorcmd.h
///
/// \brief 	All functions needed for Motorcommand-Interpretation
///
/// \author Adam Cwientzek Pascal Langenberg
///
/// \version 0.9
///
/// \date 3.08.2006
///
/// \note none
///
//////////////////////////////////////////////////////////////////////////////

#ifndef _AIS_MOTORCMD_H_
#define _AIS_MOTORCMD_H_

#include "ais_typedef.h"


// ---------------------------------
// Error Response				0xxx/0x59


// ---------------------------------
// Controller configuration		0x56/0x57


// ---------------------------------
// Motor Control Commands		0x52/0x53

#define _SET_ALL_PWM_			0x10
#define _SET_ONE_PWM_	    	0x11
#define _SET_ALL_RPM_			0x20
#define _SET_ONE_RPM_			0x21
#define _SET_DIGITAL_IO_		0x41
#define _SET_ALL_TICKS_ABS_CLEAR_ 	0x65
#define _SET_MODE_WITH_BUMPER_ 	0x70
// ---------------------------------

// ---------------------------------
// Motor Config Commands		0x50/0x51

#define _MCMD_CONTROLLER_		0x10
#define _MCMD_USE_PWM_	    	0x11
#define _MCMD_LIMITER_ 			0x12
#define _MCMD_TIMEOUT_			0x13
#define _MCMD_MAX_CUR_			0x20
#define _MCMD_MAX_RPM_			0x21
#define _MCMD_NOM_CUR_			0x22
#define _MCMD_NOM_RPM_			0x23
#define _MCMD_GEAR_RATIO_		0x30
#define _MCMD_WHEEL_RADIUS_		0x31
#define _MCMD_AXE_LENGTH_		0x32
#define _MCMD_TICKS_ROT_		0x33
#define _MCMD_SPEC_TORQUE_		0x34
#define _MCMD_DIRECTION_		0x35
#define _MCMD_MAX_TEMP_			0x40
#define _MCMD_NOM_TEMP_			0x41
#define _MCMD_ENV_TEMP_			0x42
#define _MCMD_WINDING_T_		0x43
#define _MCMD_WINDING_GN_		0x44
#define _MCMD_WINDING_GZ_		0x45
#define _MCMD_CHASSIS_T_		0x46
#define _MCMD_CHASSIS_GN_		0x47
#define _MCMD_CHASSIS_GZ_		0x48
#define _MCMD_SAVE_CONFIG_ 		0x50
#define _MCMD_LOAD_CONFIG_ 		0x51
#define _MCMD_CONFIG_IOPORT_    0x60
//#define _MCMD_Digital_Output_ 	0x61
//#define _CLEAR_ALL_TICKS_ABS_ 	0x65
#define _CLEAR_ONE_TICKS_ABS_ 	0x66

// ---------------------------------

// ---------------------------------
// Motor Status Commands		0x54/0x55

#define _SREG_MOT_RPM_		 	0x10
#define _SREG_MOT_PWM_		 	0x11
#define _SREG_MOT_VOLTAGE_	 	0x12
#define _SREG_MOT_CURRENT_ 	 	0x13
#define _SREG_MOT_TEMP_		 	0x14
#define _SREG_MOT_TORQUE_	 	0x15
#define _SREG_MOT_POWER_	 	0x16
#define _SREG_MOT_TICKS_ABS_ 	0x21
#define _SREG_MOT_TICKS_REL_ 	0x20
#define _SREG_BATTERY_ 		 	0x30
#define _SREG_Digital_Input_ 	0x40
//#define _SREG_DIGITAL_OUTPUT_ 	0x41
#define _SREG_Analog1_Input_ 	0x42
#define _SREG_Analog2_Input_ 	0x43
#define _SREG_USER_REQUEST_  	0xF0
// ---------------------------------




// ---------------------------------

// ---------------------------------
// Motor Error Commands
#define _UNKNOWN_ERR_			0x01
#define _PARAM_ERR_	    		0x10
#define _RANGE_ERR_				0x11


// ---------------------------------
// ---------------------------------
// Parameter Ranges
#define _MAX_CUR_				8000
#define _MAX_RPM_				25000
#define _NOM_CUR_				8000
#define _NOM_RPM_				25000
#define _GEAR_RATIO_			10000
#define _WHEEL_RADIUS_			1000
#define _AXE_LENGTH_			1000
#define _TICKS_ROT_				5000
#define _SPEC_TORQUE_			1000
#define _MAX_TEMP_				200
#define _NOM_TEMP_				200
#define _ENV_TEMP_				100
#define _WINDING_T_				500
#define _WINDING_GN_			100
#define _WINDING_GZ_			1000
#define _CHASSIS_T_				500
#define _CHASSIS_GN_			100
#define _CHASSIS_GZ_			1000
#define _CONFIG_				4
#define _TIMEOUT_				1000

// ---------------------------------


void ais_mctrl_config(struct CmdStruct Command);
void ais_mctrl_motorcontrol(struct CmdStruct Command);
void ais_mctrl_status1(struct CmdStruct Command);
void ais_mctrl_shortStatusRequest(VMC_UCHAR_8 channelId, VMC_UCHAR_8 requestID);

void make_status_command(struct CmdStruct *ResponseCmd, VMC_UCHAR_8 id, VMC_UCHAR_8 requestID, VMC_UCHAR_8 *param);

void ERR_range(struct CmdStruct *ErrResponse, char cmdGrp);
void ERR_unknown(struct CmdStruct *ErrResponse, char cmdGrp);
void ERR_param(struct CmdStruct *ErrResponse, char cmdGrp);

void Clear_all_Rots(void);
void Clear_ref_Rots(VMC_UCHAR_8 id);
void Switch_bumper_mode(void);


#endif /* _AIS_MOTORCMD_H_ */

//@}