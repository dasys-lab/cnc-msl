//////////////////////////////////////////////////////////////////////////////
/// \defgroup comasc Serial communication Module
//@{
/// \file ais_asccom.h
///
/// \brief 	Serial communication Module
///
/// \author Adam Cwientzek
///
/// \version 0.8
///
/// \date 30.10.2006
///
//////////////////////////////////////////////////////////////////////////////


#include <reg167.h>
#include <intrins.h>
#include "comasc/ais_asccom.h"
#include "comasc/ais_asc0.h"
#include "comasc/ais_ascbuff.h"
#include "system/ais_cmdbuff.h"
#include "system/ais_astring.h"


char SendedPackets;

//////////////////////////////////////////////////////////////////////////////
void init_asccom() {
	asc0_init();
	abuf_init();
	SendedPackets = 0;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// Start transmission
/// all =1 => all cmd in buffer, all=0 => only one next cmd
void asccom_senddata(char all) {
	asc0_start_trx(all);
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
// check buffer for overrun 
char asccom_buffer_overrun(char buffer) {
	return abuf_isoverrun(buffer);
}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// reset buffer overrun flag
char asccom_buffer_r_overrun(char buffer) {
	return abuf_r_overrun(buffer);
}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// disable recieve interrupt
void asccom_disable_rcv() {
	// return ! buffer empty ???
	S0RIC = 0;
}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// enable recieve interrupt
void asccom_enable_rcv() {
	// return ! buffer empty ???
	S0RIC = 1;
}
//////////////////////////////////////////////////////////////////////////////


void asccom_empty_buffer(char buffer) {
 abuf_delete(buffer);
}



//////////////////////////////////////////////////////////////////////////////
/// check if new command in buffer
char asccom_newcmd() {
	// return ! buffer empty ???
	return !abuf_isempty(ASC_BUF_IN);
}
//////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////////
/// get next command from buffer
char asccom_getcmd(struct CmdStruct *Cmd) {
  char valid = 1;
  unsigned char pos = 0;
  // if buffer not empty
  // read next buffer element and copy data to cmd
  // return 1 if success (cmd valid)
  abuf_moveread(ASC_BUF_IN);   	

  if ( !abuf_startread(ASC_BUF_IN) ) return 0; // could not read new data

  // note: data within IN buffer is allready without quote characters !
  (*Cmd).rxaddr  = 255; // Broadcast 
  (*Cmd).txaddr  = 255; // Broadcast
  (*Cmd).channel = _CMDB_CH_ASC0_; // Channel Nr. for ASC
  (*Cmd).sysinf  = _CMDB_ERR_OK_; // No Error in packet

  (*Cmd).count    = abuf_readbyte(ASC_BUF_IN); // First Byte is Paketcounter 
  (*Cmd).cmdgrp   = abuf_readbyte(ASC_BUF_IN); // Command Group
  (*Cmd).cmd      = abuf_readbyte(ASC_BUF_IN); // Command
	
  // message len in buffer - head size - crc byte is len of data
  // check if messag elong enough ! (otherwise msg is not valid)
  if ( abuf_read_msglen(ASC_BUF_IN) < (ASC_HEAD_SIZE+1) ) return 0; 

  (*Cmd).datalen  = abuf_read_msglen(ASC_BUF_IN) - ASC_HEAD_SIZE - 1;

  for ( pos=0; pos<(*Cmd).datalen; pos++ ) {
   (*Cmd).data[pos] = abuf_readbyte(ASC_BUF_IN);
  }
 
  abuf_stopread(ASC_BUF_IN);

  return valid;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Write character to buffer, quote if necessary
//////////////////////////////////////////////////////////////////////////////
void asccom_quotewrite(char buffer, char data) {
 if ( (data == _ASCCH_START_) || (data == _ASCCH_END_) || (data == _ASCCH_QUOTE_) )  
   	abuf_writebyte(buffer, _ASCCH_QUOTE_);

 abuf_writebyte(buffer, data);
}
//////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////////
// sotre command to buffer for sending MAIN entry
///////////////////////////////////////////////////////////////////////////////
char asccom_storecmd(struct CmdStruct *Cmd) {

 	if (Cmd->vmc_cmd_type == _CMD_RS232_) return RS232_asccom_storecmd( Cmd );

	return 1;
}

char RS232_asccom_storecmd(struct CmdStruct *Cmd) {
 unsigned char CRC = 0;
 unsigned char pos = 0;
 // copy & convert cmd to to buffer out
 abuf_movewrite(ASC_BUF_OUT);
 // Buffer full ? signal error in writing
 if ( !abuf_startwrite(ASC_BUF_OUT) ) return 0; 

 // Write command to Buffer, Convert quoted Bytes, add Packet start/stop
 
 abuf_writebyte(ASC_BUF_OUT, _ASCCH_START_);

 asccom_quotewrite(ASC_BUF_OUT, SendedPackets);
 asccom_quotewrite(ASC_BUF_OUT, (*Cmd).cmdgrp);
 CRC = (CRC + (*Cmd).cmdgrp) % 255; // simple check byte

 asccom_quotewrite(ASC_BUF_OUT, (*Cmd).cmd);
 CRC = (CRC + (*Cmd).cmd) % 255; // simple check byte

 for ( pos=0; pos < (*Cmd).datalen; pos++) {
  asccom_quotewrite(ASC_BUF_OUT, (*Cmd).data[pos] );
  CRC = (CRC + (*Cmd).data[pos]) % 255; // simple check byte
 }

 asccom_quotewrite(ASC_BUF_OUT, CRC);

 abuf_writebyte(ASC_BUF_OUT, _ASCCH_END_);

 if ( abuf_stopwrite(ASC_BUF_OUT) ) // If writing sucessfull increase counter
	 SendedPackets = ++SendedPackets % 255; // defined Overflow at 254 
 // note ! stopwrite is necessary to close new buffer element !!!!
 return 1;
}
//////////////////////////////////////////////////////////////////////////////

// output String
void asccom_debug(VMC_UCHAR_8 *string) {
	unsigned char i = 0;
	VMC_UCHAR_8 astemp[255]; // AString

    string_to_astring(astemp, string);

	abuf_movewrite(ASC_BUF_OUT);
	if ( !abuf_startwrite(ASC_BUF_OUT) ) return; 
	abuf_writebyte(ASC_BUF_OUT, _ASCCH_START_);
	abuf_writebyte(ASC_BUF_OUT, 0x00); // cunt
	abuf_writebyte(ASC_BUF_OUT, 0x00); // cmd grp
	abuf_writebyte(ASC_BUF_OUT, 0x00); // cmd
 	for ( i=0; i < astemp[0]; i++ )	abuf_writebyte(ASC_BUF_OUT, astemp[i+1] );
	abuf_writebyte(ASC_BUF_OUT, 0x00); // CRC
	abuf_writebyte(ASC_BUF_OUT, _ASCCH_END_);
	abuf_stopwrite(ASC_BUF_OUT);
	asc0_start_trx(ASC_SEND_ALL);
	
	for(i=0; i<100; i++) {}
}






//@}