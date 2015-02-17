//////////////////////////////////////////////////////////////////////////////
/// \ingroup controller
//@{
/// \file ais_controller.h
///
/// \brief Header file for the Controller Modul
///
/// \author Adam Cwientzek
///
/// \version 0.5
///
/// \date 2.10.2006
///
//////////////////////////////////////////////////////////////////////////////

#ifndef AIS_CONTROLLER_H
#define AIS_CONTROLLER_H

#include "ais_typedef.h"



// ---------------------------------
// Controller configuration
// (group _CMDGRP_CONTROLLER_PARIN_ and _CMDGRP_CONTROLLER_PAROUT_)
#define _CTRL_PID_KP_			0x10
#define _CTRL_PID_TN_			0x11
#define _CTRL_PID_TV_			0x12
#define _CTRL_PRAMP_			0x20
#define _CTRL_NRAMP_			0x21
#define _CTRL_DEADBAND_			0x2A
// Errors
#define _CTRL_ERR_UNKNOWN_		0x10
#define _CTRL_ERR_RANGE_		0x11


// integrator limits
#define _CTRL_MAX_ISUM_			30000
#define _CTRL_MIN_ISUM_			-30000

#define _CTRL_MAX_ERR_			10000;  // limit error to 10000 RPM
#define _CTRL_MIN_ERR_			-10000; // limit error to 10000 RPM

#define _CTRL_MAX_Y_			10000;  // limit outputs to 10000 RPM
#define _CTRL_MIN_Y_			-10000; // limit outputs to 10000 RPM


// decimal places of the Paramteres
#define PARAM_ACCURACY 			100

struct CTRLSTATE {
    VMC_LONG_32 sumI;
    VMC_LONG_32 oldSetRPM;
    VMC_LONG_32 ctrlDesireRPM;
    VMC_LONG_32 lastcyletime;
	VMC_LONG_32 lastek;
};

struct CTRLPARAM {
	VMC_UCHAR_8 cntrlType;  		// What Type of Controller should be used
    VMC_INT_16  nRamp, pRamp;
    VMC_INT_16  deadband;
    VMC_DOUBLE  Kpr;
    VMC_DOUBLE  Tn;
    VMC_DOUBLE  Tv;
};


void init_controller();
void default_controller_parameters();
void ais_mctrl_reg_params(struct CmdStruct Command);

void reset_controller(VMC_UCHAR_8 channelID);
VMC_INT_16 controller(VMC_UCHAR_8 channelID, VMC_INT_16 desiredRPM,
					  VMC_INT_16 actualRPM, VMC_INT_16 stellerDiff,
					  VMC_INT_16 maxRPM, TMC_ULONG_32 last_cycle_length);


VMC_INT_16 ctrl_get_nRamp(VMC_UCHAR_8 channelID);
VMC_INT_16 ctrl_set_nRamp(VMC_UCHAR_8 channelID, VMC_INT_16 value);

VMC_INT_16 ctrl_get_pRamp(VMC_UCHAR_8 channelID);
VMC_INT_16 ctrl_set_pRamp(VMC_UCHAR_8 channelID, VMC_INT_16 value);

VMC_INT_16 ctrl_get_deadband(VMC_UCHAR_8 channelID);
VMC_INT_16 ctrl_set_deadband(VMC_UCHAR_8 channelID, VMC_INT_16 value);

VMC_INT_16 ctrl_get_Kpr(VMC_UCHAR_8 channelID);
VMC_INT_16 ctrl_set_Kpr(VMC_UCHAR_8 channelID, VMC_INT_16 value);

VMC_INT_16 ctrl_get_Tn(VMC_UCHAR_8 channelID);
VMC_INT_16 ctrl_set_Tn(VMC_UCHAR_8 channelID, VMC_INT_16 value);

VMC_INT_16 ctrl_get_Tv(VMC_UCHAR_8 channelID);
VMC_INT_16 ctrl_set_Tv(VMC_UCHAR_8 channelID, VMC_INT_16 value);



#endif /* AIS_CONTROL_H */

//@}
