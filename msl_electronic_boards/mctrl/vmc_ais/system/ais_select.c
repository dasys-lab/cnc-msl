//////////////////////////////////////////////////////////////////////////////
/// \defgroup select Command Group Selector / Application Selector
//@{
/// \file ais_select.c
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



#include <stdio.h>
#include <reg167.h>
#include <INTRINS.H>
#include <stdlib.h>
#include <intrins.h>
#include <string.h>

#include "comasc/ais_asccom.h"
#include "system/ais_select.h"
#include "system/ais_cmdbuff.h"
#include "system/ais_system.h"
#include "can_stuff/ais_can.h"


// Modules

#include "motorctrl/ais_motorcmd.h"
#include "controller/ais_controller.h"



//////////////////////////////////////////////////////////////////////////////
/// Proceed all Commands in actual Buffer
///
/// \return 1 if at least one Command was proceed
///			0 if no Command was proceed
///
///
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 sel_proceed_allcmd() {
 VMC_UCHAR_8 temp = 0;

 // while buffer not empty call: sel_proceed_cmd()
 while ( !cmdb_isempty(_CMDB_RX_ ) )
  temp += sel_proceed_cmd();

 return (temp > 0);
}
//////////////////////////////////////////////////////////////////////////////






//////////////////////////////////////////////////////////////////////////////
/// Proceed one (oldest) Command in actual Buffer (source is RS232)
///
/// \return
///	
///
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 sel_proceed_cmd() {
 struct CmdStruct Command;
 // get one command from Buffer
 if ( cmdb_get_command( &Command ) ) {  Command.vmc_cmd_type = _CMD_RS232_;
										return all_sel_proceed_cmd(Command);
									 }
 return 0;
}

/* This is the main entry for comand handling. Several sources prepare the command content.
	For example RS232, can
*/
VMC_UCHAR_8 all_sel_proceed_cmd(struct CmdStruct Command) {


 // Switch with grp
  switch ( Command.cmdgrp ) {
	case _CMDGRP_BASE_: 			ais_base_proceed( Command );			// 0x10/11
						 			break;

    case _CMDGRP_SYSTEM_:   		ais_system_proceed( Command );			// 0x1A7!b
						 			break;

	// Motor Control
    case _CMDGRP_MOTOR_CONFIN_: 	ais_mctrl_config( Command );			// 0x50/51
						 			break;

    case _CMDGRP_MOTOR_CTRL_:   	ais_mctrl_motorcontrol( Command );		// 0x52/53
						 			break;

    case _CMDGRP_MOTOR_STATUSIN_:  	ais_mctrl_status1( Command );			// 0x54/55
						 			break;

    case _CMDGRP_CONTROLLER_PARIN_: ais_mctrl_reg_params( Command );		// 0x56/57
						 			break;

	// ERROR, Unknown Command
	default:			 Command.cmdgrp  = _CMDGRP_BASE_OUT_;
						 Command.cmd 	 = _BASECMD_ERR_UNKNOWN_;
 						 Command.datalen = 0;
						 cmdb_dataAppendStr(&Command,"Err ?");
						 cmdb_set_command( &Command );
						 break;
  }



 return 0;
}
//////////////////////////////////////////////////////////////////////////////



//@}
