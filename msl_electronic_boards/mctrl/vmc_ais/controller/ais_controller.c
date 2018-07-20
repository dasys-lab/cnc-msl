

#include "ais_typedef.h"
#include "aisc167b/ais_gpt.h"
#include "system/ais_utils.h"
#include "system/ais_cmdbuff.h"
#include "system/ais_select.h" // Only for Com. group defines !!

#include "controller/ais_controller.h"


#include <stdio.h>
#include <math.h>


#include "comasc/ais_asccom.h"


struct CTRLSTATE ctrlState[_NUM_MOTORS_];
struct CTRLPARAM ctrlParam[_NUM_MOTORS_];



//////////////////////////////////////////////////////////////////////////////
/// \brief Set default controller parameter
//////////////////////////////////////////////////////////////////////////////
void default_controller_parameters() {
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  VMC_UCHAR_8 channelID;
  for ( channelID = 0; channelID < _NUM_MOTORS_; channelID++ ) {
	  ctrlParam[channelID].cntrlType	= 1; // PID setter algor..

	  ctrlParam[channelID].pRamp  		= 200;
	  ctrlParam[channelID].nRamp		= -200;
	  ctrlParam[channelID].deadband		= 2;

	  ctrlParam[channelID].Kpr			= 0.3;
	  ctrlParam[channelID].Tn			= 80;
	  ctrlParam[channelID].Tv			= 0;

  }
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Initializes the controller
//////////////////////////////////////////////////////////////////////////////
void init_controller() {
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  VMC_UCHAR_8 channelID;
  for ( channelID = 0; channelID < _NUM_MOTORS_; channelID++ ) {
	  ctrlState[channelID].sumI  			= 0;
	  ctrlState[channelID].oldSetRPM		= 0;
	  ctrlState[channelID].ctrlDesireRPM	= 0;
	  ctrlState[channelID].lastcyletime		= 0;
	  ctrlState[channelID].lastek  		 	= 0;
  }
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Send a range error to the Command Buffer, quick and dirty...
//////////////////////////////////////////////////////////////////////////////
void ctrl_ERR_range(struct CmdStruct *ErrResponse, char cmdGrp) {
	ErrResponse->cmdgrp  = cmdGrp;
    ErrResponse->cmd     = _CTRL_ERR_RANGE_;
	ErrResponse->datalen = 0;
    cmdb_dataAppendStr( ErrResponse,"Parameter out of range");
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Send a unkown error to the Command Buffer, quick and dirty...
//////////////////////////////////////////////////////////////////////////////
void ctrl_ERR_unknown(struct CmdStruct *ErrResponse, char cmdGrp) {
	ErrResponse->cmdgrp  = cmdGrp;
    ErrResponse->cmd     = _CTRL_ERR_UNKNOWN_;
	ErrResponse->datalen = 0;
	cmdb_dataAppendStr(ErrResponse,"Err");
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief process controller configuration commands, call appropriate 
///        setter or getter function
/// \note  using groups _CMDGRP_CONTROLLER_PARIN_, _CMDGRP_CONTROLLER_PAROUT_
//////////////////////////////////////////////////////////////////////////////
void ais_mctrl_reg_params(struct CmdStruct Command) {
	VMC_UINT_16 param;		    // Integer parameter in command
	VMC_UINT_16 chkParam = 1;	// response parameter for chk, Default : True

	// Variable for the channel to config, (Range 1 to 3) is converted to
	VMC_UCHAR_8 channelID 		= Command.data[0] - 1; // index (Range 0 to 2)

	// Command for response
	struct CmdStruct ResponseCmd;

	// default response is error 
	ctrl_ERR_range(&ResponseCmd, _CMDGRP_CONTROLLER_PAROUT_ );

	// check if requested channel is within allowed range
	if ( !_IS_MOTORID_(channelID) ) {
		cmdb_set_command( &ResponseCmd );
		return; // return error and leave if index out of range
	}



	// switch with requested command to apropriate setter/getter function
  	switch ( Command.cmd ) {

		// set or get proportional factor
		case _CTRL_PID_KP_:
			if (Command.datalen > 1) { // Parameters available, call setter
				param = _DATA_TO_INT_(Command.data[1], Command.data[2]);	
				chkParam = ctrl_set_Kpr(channelID, param);
				
			}
			if ( chkParam ) { // If no setter or setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, ctrl_get_Kpr(channelID) ); // Append Data				
			}
			break;

		// set or get integrativ part parameter
		case _CTRL_PID_TN_:
			if (Command.datalen > 1) {// Parameters available, call setter
				param = _DATA_TO_INT_(Command.data[1], Command.data[2]);
				chkParam = ctrl_set_Tn(channelID, param);
			}
			if ( chkParam ) { // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, ctrl_get_Tn(channelID) ); // Append Data
			}
			break;

		// set or get der. part parameter
		case _CTRL_PID_TV_:
			if (Command.datalen > 1) {// Parameters available, call setter
				param = _DATA_TO_INT_(Command.data[1], Command.data[2]);
				chkParam = ctrl_set_Tv(channelID, param);
			}
			if ( chkParam ) { // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, ctrl_get_Tv(channelID) ); // Append Data
			}
			break;

		// set or get positiv ramp
		case _CTRL_PRAMP_:
			if (Command.datalen > 1) {// Parameters available, call setter
				param = _DATA_TO_INT_(Command.data[1], Command.data[2]);
				chkParam = ctrl_set_pRamp(channelID, param);
			}
			if ( chkParam ) { // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, ctrl_get_pRamp(channelID) ); // Append Data
			}
			break;

		// set or get negativ ramp
		case _CTRL_NRAMP_:
			if (Command.datalen > 1) {// Parameters available, call setter
				param = _DATA_TO_INT_(Command.data[1], Command.data[2]);
				chkParam = ctrl_set_nRamp(channelID, param);
			}
			if ( chkParam ) { // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, ctrl_get_nRamp(channelID) ); // Append Data
			}
			break;

		// set or get deadband 
		case _CTRL_DEADBAND_:
			if (Command.datalen > 1) {// Parameters available, call setter
				param = _DATA_TO_INT_(Command.data[1], Command.data[2]);
				chkParam = ctrl_set_deadband(channelID, param);
			}
			if ( chkParam ) { // If no setter of setter succeeded answer with new Param
				ResponseCmd.datalen = 1; // Delete error message
				ResponseCmd.data[0] = channelID + 1; // Return also channel !
				cmdb_dataAppendInt( &ResponseCmd, ctrl_get_deadband(channelID) ); // Append Data
			}
			break;
		
		// return unknown error if unknown command
    	default: ctrl_ERR_unknown(&ResponseCmd, _CTRL_ERR_UNKNOWN_); // Send unknown Error Message !
  	}

	// if a setter was called, get back the new values to create check response with
	// new parameters
    if ( chkParam ) { // No error occurred, prepare usual response
		ResponseCmd.cmdgrp = _CMDGRP_CONTROLLER_PAROUT_; // Grp is "motor config"
		ResponseCmd.cmd    = Command.cmd;    // Same as Request Command
	}

	// Send command to command buffer
	cmdb_set_command( &ResponseCmd );
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Initializes the controller
//////////////////////////////////////////////////////////////////////////////
void reset_controller(VMC_UCHAR_8 channelID) {
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if ( _IS_MOTORID_(channelID) ) {
	  ctrlState[channelID].sumI = 0;
  }
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief local function calculates ramp for deltaDesire speed
//////////////////////////////////////////////////////////////////////////////
VMC_INT_16 ctrl_calc_ramp( VMC_UCHAR_8 channelID, VMC_INT_16 deltaDesire ) {
 	// calculate ramp. If deltaDesire is bigger than max. allowed step
	// then reduce desiredRPM for this cycle.
	// Use apropriate ramp step (negativ ramp / positiv ramp)
	if ( deltaDesire > 0 ) { // use positiv Ramp
	  if ( ctrlParam[channelID].pRamp > 0 ) { // pRamp active ?
	   return MIN(ctrlParam[channelID].pRamp, deltaDesire);
	  } else { // Ramp not active
	   return deltaDesire;
	  }
	} 
	
	if ( deltaDesire < 0 ) {  // use negativ Ramp
	  if ( ctrlParam[channelID].nRamp < 0 ) { // pRamp active ?
	   return MAX(ctrlParam[channelID].nRamp, deltaDesire);
	  } else { // Ramp not active
	   return deltaDesire;
	  }
	}
	return 0;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief local function calculate controller deviation
//////////////////////////////////////////////////////////////////////////////
VMC_FLOAT ctrl_calc_error(VMC_INT_16 desired, VMC_INT_16 actualRPM) {
    VMC_FLOAT ek  			  	= 0; // controller deviation 
	ek 	   = desired - actualRPM;
	// check for overflow
	switch ( _chkfloat_(ek) ) {
	  case 2: ek = _CTRL_MAX_ERR_; break;
	  case 3: ek = _CTRL_MIN_ERR_; break;
	  case 4: ek = 0.0; break;  
	  default:;
	}
	return ek;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief local function calculate proportional part of controllrt
//////////////////////////////////////////////////////////////////////////////
VMC_FLOAT ctrl_calc_P(VMC_UCHAR_8 channelID, VMC_FLOAT err) {
    VMC_FLOAT y; // temp output	
	// Proportional part
	y = err * ctrlParam[channelID].Kpr;
	// check for overflow
	switch ( _chkfloat_(y) ) {
	  case 2: y = _CTRL_MAX_Y_; break;
	  case 3: y = _CTRL_MIN_Y_; break;
	  case 4: y = 0.0; break;	  
	  default: ;
	}	
	return y;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief local function calculate integrative part of controllrt
//////////////////////////////////////////////////////////////////////////////
VMC_FLOAT ctrl_calc_I(VMC_UCHAR_8 channelID, VMC_FLOAT err, VMC_INT_16 stellerDiff, VMC_LONG_32 TA) {
    VMC_FLOAT y; // temp output

	// Integrate with anti-windup:
	ctrlState[channelID].sumI += err - stellerDiff;

	// Limit Integrator
	if ( ctrlState[channelID].sumI > _CTRL_MAX_ISUM_ ) ctrlState[channelID].sumI = _CTRL_MAX_ISUM_;
	if ( ctrlState[channelID].sumI < _CTRL_MIN_ISUM_ ) ctrlState[channelID].sumI = _CTRL_MIN_ISUM_;

	// calculate integrativ part
	if ( (ctrlParam[channelID].Tn > 0.0) ) {
	 y = ctrlParam[channelID].Kpr * (TA / ctrlParam[channelID].Tn);
 	 y = y * ctrlState[channelID].sumI;	 
    } else {
	 y = 0.0;
	}

	// check for overflow
	switch ( _chkfloat_(y) ) {
	  case 2: y = _CTRL_MAX_Y_; break;
	  case 3: y = _CTRL_MIN_Y_; break;
	  case 4: y = 0.0; break;	  
	  default: ;
	}	
	return y;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief local function calculate Derivative part of controllrt
//////////////////////////////////////////////////////////////////////////////
VMC_FLOAT ctrl_calc_D(VMC_UCHAR_8 channelID, VMC_FLOAT err, VMC_INT_16 last_err, VMC_LONG_32 TA) {
    VMC_FLOAT y; // temp output
	// Derivative part :::::::::::::::::::::::::::::::::::::::::::::::::::
	y = ctrlParam[channelID].Kpr * (ctrlParam[channelID].Tv / TA);
	y = y * (err - last_err);

    // Derivative part end :::::::::::::::::::::::::::::::::::::::::::::::
	// check for overflow
	switch ( _chkfloat_(y) ) {
	  case 2: y = _CTRL_MAX_Y_; break;
	  case 3: y = _CTRL_MIN_Y_; break;
	  case 4: y = 0.0; break;	  
	  default: ;
	}	

	//return 0.0; // deactivate D-Part
	return y;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief the controller: PID with anti-windup and ramp function
// "real" PID in additiv form
//////////////////////////////////////////////////////////////////////////////
VMC_INT_16 controller(VMC_UCHAR_8 channelID, VMC_INT_16 desiredRPM,
					  VMC_INT_16 actualRPM, VMC_INT_16 stellerDiff,
					  VMC_INT_16 maxRPM, TMC_ULONG_32 last_cycle_length) {
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// temporary varibles ----------------------------------------------------
	VMC_LONG_32 TA  			= 0; // sample time in ms
	VMC_LONG_32 lastek 			= 0; // deviation in last cycle
	VMC_LONG_32 actualTime 		= 0;
	VMC_INT_16  deltaDesire 	= 0;

	VMC_FLOAT ek  			  	= 0; // controller deviation
	VMC_FLOAT ypk 				= 0; // proportional part
	VMC_FLOAT yik 				= 0; // itegrativ part
	VMC_FLOAT ydk 				= 0; // derativ part
	VMC_LONG_32 yrk 			= 0; // PID output sum
	// -----------------------------------------------------------------------


	// Range check for channelID !!
	if ( !_IS_MOTORID_(channelID) ) return 0;
	
	// Calculate difference to desired RPM in last cycle
	deltaDesire = desiredRPM - ctrlState[channelID].ctrlDesireRPM;

	// If deltaDesire bigger that ramp, limit to maximum ramp value
    ctrlState[channelID].ctrlDesireRPM += ctrl_calc_ramp(channelID, deltaDesire);

	// --- DEBUG INFORMATION ---------------------------------------------
	// now ctrlDesireRPM should be either desiredRPM or growing towards it
	// --- DEBUG INFORMATION ---------------------------------------------

	// calculate sample time
	// get actual runtime since main loop start in [us]
	actualTime = get_cycle_time_part();
	// sample time is time since "main" - "loop start" + "time after last"
	TA = (last_cycle_length - ctrlState[channelID].lastcyletime) + actualTime;
	// Now TA hast time in us, calculate into ms
    TA = (VMC_INT_16)floor(TA / 1000);


	// calculate controller deviation
	ek 	   = ctrl_calc_error(ctrlState[channelID].ctrlDesireRPM, actualRPM);

	// for calculation of deravativ part we need also last deviation
	// get deviation in last cycle
	lastek = ctrlState[channelID].lastek;

	// store actual deviation for next cycle
	ctrlState[channelID].lastek = ek;

	// If Error within deadband deactivate Controller, return last calculated value
	//if ( (ek < ctrlParam[channelID].deadband) && (ek > ( -1 * ctrlParam[channelID].deadband) ) )
	//	return ctrlState[channelID].oldSetRPM;


	// --- DEBUG INFORMATION ---------------------------------------------
	// The PID Algo. starts here !
	// --- DEBUG INFORMATION ---------------------------------------------


	// Proportional part :::::::::::::::::::::::::::::::::::::::::::::::::
	ypk = ctrl_calc_P(channelID, ek);

	// Integrativ part :::::::::::::::::::::::::::::::::::::::::::::::::::	
	yik = ctrl_calc_I(channelID, ek, stellerDiff, TA);

	// Derivative part :::::::::::::::::::::::::::::::::::::::::::::::::::
	ydk = ctrl_calc_D(channelID, ek, lastek, TA);

	// PID sum
	yrk = floor(ypk + yik + ydk); 


	// Limit controller output to max RPM
	if ( yrk > maxRPM ) yrk = maxRPM;
	if ( yrk < (maxRPM * -1) ) yrk = maxRPM * -1;

	// Remember controller output for next period
	ctrlState[channelID].oldSetRPM = yrk;

	return (VMC_INT_16)yrk; //typecast long to int 16
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// Setter/Getter Functions for controller Parameters
//////////////////////////////////////////////////////////////////////////////


VMC_INT_16 ctrl_get_nRamp(VMC_UCHAR_8 channelID) {
    if (  !_IS_MOTORID_(channelID) ) return 0;
	return ctrlParam[channelID].nRamp * -1;
}

VMC_INT_16 ctrl_set_nRamp(VMC_UCHAR_8 channelID, VMC_INT_16 value) {
    if (  !_IS_MOTORID_(channelID) ) return 0;
	ctrlParam[channelID].nRamp = -1 * value;
	return 1;
}

VMC_INT_16 ctrl_get_pRamp(VMC_UCHAR_8 channelID) {
    if (  !_IS_MOTORID_(channelID) ) return 0;
	return ctrlParam[channelID].pRamp;
}

VMC_INT_16 ctrl_set_pRamp(VMC_UCHAR_8 channelID, VMC_INT_16 value) {
    if (  !_IS_MOTORID_(channelID) ) return 0;
	ctrlParam[channelID].pRamp = value;
	return 1;
}

VMC_INT_16 ctrl_get_deadband(VMC_UCHAR_8 channelID) {
    if (  !_IS_MOTORID_(channelID) ) return 0;
	return ctrlParam[channelID].deadband;
}

VMC_INT_16 ctrl_set_deadband(VMC_UCHAR_8 channelID, VMC_INT_16 value) {
    if (  !_IS_MOTORID_(channelID) ) return 0;
	ctrlParam[channelID].deadband = value;
	return 1;
}

VMC_INT_16 ctrl_get_Kpr(VMC_UCHAR_8 channelID) {
    if (  !_IS_MOTORID_(channelID) ) return 0;
	return floor(PARAM_ACCURACY * ctrlParam[channelID].Kpr);
}

VMC_INT_16 ctrl_set_Kpr(VMC_UCHAR_8 channelID, VMC_INT_16 value) {
    if (  !_IS_MOTORID_(channelID) ) return 0;
	ctrlParam[channelID].Kpr = (VMC_DOUBLE)value / PARAM_ACCURACY;
	return 1;
}

VMC_INT_16 ctrl_get_Tn(VMC_UCHAR_8 channelID) {
    if (  !_IS_MOTORID_(channelID) ) return 0;
	return floor(PARAM_ACCURACY * ctrlParam[channelID].Tn);
}

VMC_INT_16 ctrl_set_Tn(VMC_UCHAR_8 channelID, VMC_INT_16 value) {
    if (  !_IS_MOTORID_(channelID) ) return 0;
	ctrlParam[channelID].Tn = (VMC_DOUBLE)value / PARAM_ACCURACY;
	return 1;
}

VMC_INT_16 ctrl_get_Tv(VMC_UCHAR_8 channelID) {
    if (  !_IS_MOTORID_(channelID) ) return 0;
	return floor(PARAM_ACCURACY * ctrlParam[channelID].Tv);
}

VMC_INT_16 ctrl_set_Tv(VMC_UCHAR_8 channelID, VMC_INT_16 value) {
    if (  !_IS_MOTORID_(channelID) ) return 0;
	ctrlParam[channelID].Tv = (VMC_DOUBLE)value / PARAM_ACCURACY;
	return 1;
}
