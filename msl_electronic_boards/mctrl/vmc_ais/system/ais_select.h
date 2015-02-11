//////////////////////////////////////////////////////////////////////////////
/// \defgroup select Command Group Selector / Application Selector
//@{
/// \file ais_select.h
///
/// \brief 	Selects the appropriate Application Interpreter for given
///			Command Group, prepares Command for Interpreter
///
/// \author Adam Cwientzek
///
/// \version 0.8
///
/// \date 27.12.2005
///
/// \note none
///
//////////////////////////////////////////////////////////////////////////////

#ifndef _AIS_SELECT_H_
#define _AIS_SELECT_H_

#include "ais_typedef.h"

// Command Groups, Modules and Functions

// BASE: Basic commandos like version, echo etc.
#define _CMDGRP_BASE_   			0x10
#define _CMDGRP_BASE_OUT_			0x11

// SYSTEM: System related calls like Battary Voltage etc.
#define _CMDGRP_SYSTEM_  			0x20
#define _CMDGRP_SYSTEM_OUT_			0x21


// ---------------------------------
// Motor Control Groups
#define _CMDGRP_MOTOR_CONFIN_		0x50
#define _CMDGRP_MOTOR_CONFOUT_		0x51
#define _CMDGRP_MOTOR_CTRL_			0x52
#define _CMDGRP_MOTOR_STATUSIN_		0x54
#define _CMDGRP_MOTOR_STATUSOUT_	0x55
#define _CMDGRP_CONTROLLER_PARIN_	0x56
#define _CMDGRP_CONTROLLER_PAROUT_	0x57
#define _CMDGRP_MOTOR_ERR_			0x59
#define _CMDGRP_MOTOR_MASK_			0x50
// ---------------------------------




unsigned char sel_proceed_allcmd();
unsigned char sel_proceed_cmd();
VMC_UCHAR_8 all_sel_proceed_cmd(struct CmdStruct Command);


#endif /* _AIS_SELECT_H_ */

//@}