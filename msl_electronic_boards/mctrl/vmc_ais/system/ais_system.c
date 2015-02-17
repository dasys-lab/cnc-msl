//////////////////////////////////////////////////////////////////////////////
/// \defgroup System Module for AISC167 Board
//@{
/// \file ais_system.h
///
/// \brief 	Various System Functions / System configuration like Cycle Time etc.
///
/// \author Adam Cwientzek
///
/// \version 0.8
///
/// \date 31.12.2005
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


#include "ais_typedef.h"
#include "system/ais_system.h"
#include "system/ais_select.h"
#include "system/ais_cmdbuff.h"


// Basic System Functions like Echo, Version etc.
void ais_base_proceed(struct CmdStruct Command) {
 
 switch ( Command.cmd ) {
   case _BASECMD_ECHO_: cmdb_set_command( &Command ); 
						 break;						 	
   case _BASECMD_VER_:  Command.datalen = 0; // Empty data in command frame
   						 cmdb_dataAppendStr(&Command,_FIRMWWARE_VER_);
						 cmdb_set_command( &Command );
						 break;
   
 } // switch
}



void ais_system_init() {
 struct CmdStruct Command;
 Command.cmdgrp  = _CMDGRP_BASE_OUT_;
 Command.cmd 	 = _BASECMD_VER_;
 Command.datalen = 0;
 Command.data[0] = 0;
 ais_base_proceed(Command); // Call Echo output !
}


// Periodic System Function
void ais_system_periodic() {
 // Nothing usefull yet 
}

void ais_system_bufferError() {
 struct CmdStruct Command;
 Command.cmdgrp  = _CMDGRP_BASE_OUT_;
 Command.cmd 	 = _BASECMD_ERR_BUFFER_;
 Command.datalen = 0;
 Command.data[0] = 0;

 cmdb_dataAppendStr(&Command,"ERR: BuffOVR");
 cmdb_set_command( &Command );
}


// Procceed System related Commands ( Battery etc.)
void ais_system_proceed(struct CmdStruct Command) { 
	Command.cmd = 0; // Nothing relevant yet
}


//@}
