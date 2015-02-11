//////////////////////////////////////////////////////////////////////////////
/// \defgroup Communication Buffer for AISC167Board
//@{
/// \file ais_combuff.c
///
/// \brief 	Communication Buffer and Fetch&Validate Functions for AISC167
/// 		Communication
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

#include "system/ais_cmdbuff.h"
#include "system/ais_astring.h"
#include "comasc/ais_asccom.h"
#include "system/ais_system.h"
#include "can_stuff/ais_can.h"

#include "motorctrl/ais_error.h" // needed to reset timeout - bad solution !!


// Buffer
struct CmdStruct cmdb_RX[_CMDB_RX_SIZE_];
struct CmdStruct cmdb_TX[_CMDB_RX_SIZE_];

// Index for Buffer, see _CMDB_LAST_/_CMDB_FIRST_ and
// _CMDB_RX_ / _CMDB_TX_
VMC_UCHAR_8 cmdb_index[2][2] = { {0,0}, {0,0} };




// ################ Buffer PointerControl ####################################



//////////////////////////////////////////////////////////////////////////////
/// Calucalte/Move Buffer-Indexes
///
/// \param Buffer use constants to select Buffer : _CMDB_RX_ or _CMDB_TX_
///
/// \param Index use constants to select Index : _CMDB_FIRST_ or _CMDB_LAST_
///
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 cmdb_move_next(VMC_UCHAR_8 Buffer, VMC_UCHAR_8 Index) {
 VMC_UINT_16 TmpIdx;
 // Get Index Value
 TmpIdx = cmdb_index[Buffer][Index] + 1;

 // Modulo Buffersize (ring Buffer) Differ Buffer, take appropriate size
 if ( Buffer == _CMDB_RX_ ) {
   TmpIdx = TmpIdx % _CMDB_RX_SIZE_;
 } else {
   TmpIdx = TmpIdx % _CMDB_TX_SIZE_;
 }

 switch ( Index ) {
  // For LAST Index:
  // when both Indexes are same, that means we have reached first entry again.
  // Buffer is full, first entry (oldest) will be discarded!
  case _CMDB_LAST_:

				if ( TmpIdx == cmdb_index[Buffer][_CMDB_FIRST_] )
			    cmdb_move_next(Buffer, _CMDB_FIRST_); // Move first, discard oldest Command!!
				break;


  // For Index FIRST:
  // Check if any Data is available, else do not move Pointer!
  case _CMDB_FIRST_:

				if ( cmdb_index[Buffer][_CMDB_FIRST_] == cmdb_index[Buffer][_CMDB_LAST_] )
			  	return 0;

 }

 cmdb_index[Buffer][Index] = TmpIdx;
 return 1;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// Count entries if RX Buffer
///
/// \return
///
///
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 cmdb_count(VMC_UCHAR_8 Buffer) {
 // diff. between First ans last Position
 int count = cmdb_index[Buffer][_CMDB_LAST_] - cmdb_index[Buffer][_CMDB_FIRST_];

 // needed for overflow in Index (Ring-Buffer!)
 if ( count < 0 )
  switch ( Buffer ) {
    case _CMDB_RX_ : count += _CMDB_RX_SIZE_;
					 break;

    case _CMDB_TX_ : count += _CMDB_TX_SIZE_;
					 break;
  }


 return count;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// Check if RX Buffer is empty
///
/// \return
///
///
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 cmdb_isempty(VMC_UCHAR_8 Buffer) {
 return (cmdb_count(Buffer) == 0);
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// Copy Data from Cmd1 to Cmd2
///
//////////////////////////////////////////////////////////////////////////////
void cmdb_CommandCopy(struct CmdStruct *Cmd1, struct CmdStruct *Cmd2) {
 VMC_UINT_16 i;

 (*Cmd2).rxaddr 	= (*Cmd1).rxaddr;
 (*Cmd2).txaddr 	= (*Cmd1).txaddr;
 (*Cmd2).count 		= (*Cmd1).count;
 (*Cmd2).sysinf 	= (*Cmd1).sysinf;
 (*Cmd2).datalen 	= (*Cmd1).datalen;
 (*Cmd2).channel 	= (*Cmd1).channel;
 (*Cmd2).cmdgrp 	= (*Cmd1).cmdgrp;
 (*Cmd2).cmd 		= (*Cmd1).cmd;

 for ( i=0; i<_CMDB_MAXDATA_; i++ ) {
  (*Cmd2).data[i] = (*Cmd1).data[i];
 }
}
//////////////////////////////////////////////////////////////////////////////




// ################ TRIGGER FUNCTIONS ########################################

//////////////////////////////////////////////////////////////////////////////
/// Load one (oldest) or all Command from given (or all) channel(s).
///
/// \param	Channel: The Channel to look for new Commands. See ais_combuff.h
///					 for possible constants like _CMDB_CH_*_
///					 For ASC0 use _CMDB_CH_ASC0_
///					 To read all Channels use _CMDB_CH_ALL_
///
/// \param  all:	 Use _CMDB_ALL_ if all Commands should be read
///					 Use _CMDB_ONE_ if only one (oldest) Command should be read
///
/// \return 		 1 Command was loaded
///					 0 No new Command loaded
///
///
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 cmdb_load_command(VMC_UCHAR_8 Channel, VMC_UCHAR_8 all) {
 VMC_UCHAR_8 loaded = 0;
// struct CmdStruct temp_command;

 // Get new Data from ASC0
 switch ( Channel ) {
  // Get Data from All Interfaces
  case _CMDB_CH_ALL_  :

  // Get Data from ASC0
  case _CMDB_CH_ASC0_ :
			// New Command in ASC0 Buffer ?
			if ( asccom_getcmd( &cmdb_RX[ cmdb_index[_CMDB_RX_][_CMDB_LAST_] ] ) ) {
				// New Command copied to Buffer, mark buffer space as used
				cmdb_move_next(_CMDB_RX_, _CMDB_LAST_);
 				loaded = 1; // Marke that one new command has been loaded
				reset_timeout(); 														// ? trouble ?
				if ( all ) cmdb_load_command(Channel, all); // Recursive Call
 			}
			// HANDLE BUFFER OVERRUN!
		    if ( asccom_buffer_overrun(ASC_BUF_IN) ) {			  
				   // throw away rest of commands !
 				   asccom_empty_buffer(ASC_BUF_IN);
				   // Reset overrun flag
				   asccom_buffer_r_overrun(ASC_BUF_IN); 
				   // SIGNAL OVERRUN ?
				   ais_system_bufferError();
		    } 

            // Break "switch" if single Channel selected, else go on
			if ( Channel != _CMDB_ALL_ ) break;

 }

 return loaded;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// Send one (oldest) or all Commands from Buffer
///
/// \return
///
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 cmdb_send_command(VMC_UCHAR_8 all) {
 if ( cmdb_isempty( _CMDB_TX_ ) ) {
  return 0;
 } else {
  // Get one Command from Buffer
  asccom_storecmd( &cmdb_TX[ cmdb_index[_CMDB_TX_][_CMDB_FIRST_] ] );
  // Now next Command is first Buffer Entry
  cmdb_move_next(_CMDB_TX_, _CMDB_FIRST_);
 }

 if ( all ) return cmdb_send_command(all); // Recursive Call for all Buffer entries
 return 0;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// Return oldest Command from RX Buffer
///
/// \param *Command Pointer to Command Structure where Bufferdata will be copied
///			        to
///
/// \return 1 if Data could be loaded from Buffer
///			0 if Buffer was empty (no data copied)
///
///
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 cmdb_get_command(struct CmdStruct *Command) {
 // Check if buffer is empty
 if ( cmdb_isempty( _CMDB_RX_ ) ) {
  return 0;
 } else {
  // f Data in buffer, copy it
  cmdb_CommandCopy( &cmdb_RX[ cmdb_index[_CMDB_RX_][_CMDB_FIRST_] ], Command);
  // Move buffer Index
  cmdb_move_next(_CMDB_RX_, _CMDB_FIRST_);
  // Return "Data copied"
  return 1;
 }
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// Store Command in TX Buffer
///
/// \param *Command Pointer to Command Structure that will be copied into Buf.
///
//////////////////////////////////////////////////////////////////////////////
void cmdb_set_command(struct CmdStruct *Command) {



 // Sotre Data into Send-Buffer
 cmdb_CommandCopy( Command, &cmdb_TX[ cmdb_index[_CMDB_TX_][_CMDB_LAST_] ] );
 // Move Index, overwrite old Data if full Buffer
  cmdb_move_next(_CMDB_TX_, _CMDB_LAST_);
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Add an Integer value as data to a command struct
/// This function gets a pointer to a command struct and manipulates it directly
/// It adds a Integer, split up into MSB and LSB to the data of the command and increases the datalen
/// \param Command Pointer to the command struct
/// \param param Integer value which should be add to the command
//////////////////////////////////////////////////////////////////////////////
void cmdb_dataAppendStr(struct CmdStruct *Command,VMC_UCHAR_8 *string) {
	// loop counter
	VMC_UCHAR_8 i;

	// current datalen --> offset for new data
	VMC_UCHAR_8 offset = Command->datalen;

	// temp astring
	VMC_UCHAR_8 astemp[255]; // AString
    string_to_astring(astemp, string);

	for (i = 0; i < astemp[0]; i++) {
		Command->data[i + offset] = astemp[i+1];
        Command->datalen++;
	}
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Add an Integer value as data to a command struct
/// This function gets a pointer to a command struct and manipulates it directly
/// It adds a Integer, split up into MSB and LSB to the data of the command and increases the datalen
/// \param Command Pointer to the command struct
/// \param param Integer value which should be add to the command
//////////////////////////////////////////////////////////////////////////////
void cmdb_dataAppendChar(struct CmdStruct *Command, VMC_UCHAR_8 param) {
// Function could be macro ?
	// write the next byte in the command and increase datalen
	Command->data[Command->datalen++] = param;
}
//////////////////////////////////////////////////////////////////////////////
void cmdb_dataAppendInt(struct CmdStruct *Command, VMC_UINT_16 param) {
// Function could be macro ?
	// write the MSB to the next byte in the command and increase datalen
	Command->data[Command->datalen++] = param >> 8;
	// write the LSB to the next byte in the command and increase datalen
  	Command->data[Command->datalen++] = param;
}
//////////////////////////////////////////////////////////////////////////////

void cmdb_dataAppendLong(struct CmdStruct *Command, TMC_ULONG_32 param) {
// Function could be macro ?
	// write the MSB to the next byte in the command and increase datalen
	Command->data[Command->datalen++] = param >> 24;
	Command->data[Command->datalen++] = param >> 16;
	Command->data[Command->datalen++] = param >> 8;
	// write the LSB to the next byte in the command and increase datalen
  	Command->data[Command->datalen++] = param;
}
//////////////////////////////////////////////////////////////////////////////





//@}