//////////////////////////////////////////////////////////////////////////////
/// \ingroup comasc Hardware RS232 interface
//@{
/// \file ais_asc0.c
///
/// \brief 	ASC0 Functions
///
/// \author Adam Cwientzek
///
/// \version 2.1
///
/// \date 29.12.2005
///
//////////////////////////////////////////////////////////////////////////////

#include <reg167.h>
#include <intrins.h>
#include <stdio.h>
#include <stdlib.h>
#include "system/ais_cmdbuff.h"
#include "system/ais_utils.h"

#include "comasc/ais_asc0.h"
#include "comasc/ais_ascbuff.h"
#include "comasc/ais_asccom.h"

char asc_received_data;
char asc_com_state;
char asc_trx_running;
char asc_send_all;

char send_buffer[255];
char isend_buffer;
char csend_buffer;

sbit D2OUT   = P2^14;
sbit DOUT   = P2^15;

void strobe() {
  unsigned char i=0;

  D2OUT  = 1; 
  //DOUT  = 1; 
  for (i=0; i<5; i++) ;

  D2OUT = 0; // TRIGGER
  //DOUT  = 0;
  for (i=0; i<5; i++) ;
  D2OUT = 1;
  //DOUT  = 1;
  for (i=0; i<5; i++) ;
}




//////////////////////////////////////////////////////////////////////////////
/// Initialize ASC0 Interface. 
/// 
/// Must be called beofre ASC0 can be used.
/// Current Settings: 57600 BAUD, 1 Stop Bit, no Parity
///
//////////////////////////////////////////////////////////////////////////////
void asc0_init(void) {
   P3  |= 0x0400;        // SET PORT 3.10 OUTPUT LATCH (TXD)
   DP3 |= 0x0400;        // SET PORT 3.10 DIRECTION CONTROL (TXD OUTPUT)
   DP3 &= 0xF7FF;        // RESET PORT 3.11 DIRECTION CONTROL (RXD INPUT)

   asc_received_data = 0;	 // No character read 
   asc_com_state 	 = _ASCST_WAIT_; // goto State normal Communication     
   asc_send_all		 = 0;

   S0BG  = 0x0A;         // SET BAUDRATE TO 57600 BAUD                     
   S0CON = 0x8011;       // SET SERIAL MODE                               

   S0RIC 			= 0x45;	 // enable asc0 recive interrupt

   asc_trx_running 	= 0;
   csend_buffer		= 0;
   isend_buffer     = 1;

   S0TBIC 		   	= 0x04; // ctrl Register buffer interrupt
   S0TIC			= 0x04; // Transmit irq ctrl.
   S0EIC 			= 0x00; // error irq ctrl.   	


}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// ASC0 Interrupt handling. Read Data from ASC0 and store in temp Buffer.\n
/// When max. Data length is reached or a End-Character is read \n
/// the asc0_feed_combuffer is called.\n
///
/// \note Max. Data length can be defined in "ais_asc0.h" as _ASC_MAX_DATA_ \n
///		  The End Character is also defined there as _DATA_END_ \n
///
/// \attention Interrupt 0x2b on Adress 0xAC is used by this Function. \n
///			   When using Keil Monitor use "NMI only" in the Monitor Driver\n
///			   settings. \n
//			   This function will also recieve Monitor Data !! \n
//////////////////////////////////////////////////////////////////////////////
void asc0_rcv_interrupt(void) interrupt 0x2b 
{
  switch ( asc_com_state ) {

	// State WAIT
	//				Wait until Packet-Start is Detected
	// next: 
	//				_ASCST_START_ when Packet Start Detected
	//		
	case _ASCST_WAIT_ :			
			if (S0RBUF == _ASCCH_START_)   asc_com_state = _ASCST_START_;
			break;
	// -----------------------------------------------------------------------


	// State START
	//				Prestate of RCVD (Recieve). Prepare for new Packet
	// next:	
	//				Directly to RECIEVE, without event !
	//
	case _ASCST_START_ :			
			asc_received_data = 0;		// Reset rcv counter
			abuf_movewrite(ASC_BUF_IN); // Move write to next free element 
			if ( abuf_isoverrun(ASC_BUF_IN) ) {				
				asc_com_state = _ASCST_WAIT_;	// STOP on buffer overrun
			} else {
				abuf_startwrite(ASC_BUF_IN);	// start writing in buffer
				asc_com_state = _ASCST_RCVD_;   // next state is "RECIEVE"
			}
			// NO BREAK !!! Prestate of RCVD
	// -----------------------------------------------------------------------


	// State RECEIVE
	//				Proceed received Character
	// next:
	//				START after Packet-Start Character, (discarded act. Packet)
	//				QUOTE after Quotation was Detected
	//				WAIT  after Packetend was Detected 
	//				RECEIVE for any other Character		
	//
	case _ASCST_RCVD_ :			
    		switch ( S0RBUF ) {			 // Check for special Characters
	 		  case _ASCCH_START_: 
					  asc_com_state = _ASCST_START_; // ups, new Packet, new begin
 					  break; // Any unfinished Packet will be discarded

	 		  case _ASCCH_END_:	 
					  abuf_stopwrite(ASC_BUF_IN);    // Stop writing in buffer					  
					  asc_com_state = _ASCST_WAIT_;  // Wait for new Data
					  break;
	
	 		  case _ASCCH_QUOTE_:   
					  asc_com_state = _ASCST_QT_; // Quote Character recieved
					  break;

	 		  default:			 
			  		  abuf_writebyte(ASC_BUF_IN, S0RBUF);
					  asc_received_data++;
					  if (asc_received_data > _ASC_MAX_LEN_) asc_com_state = _ASCST_WAIT_;	
		    }			
			break;		
	// -----------------------------------------------------------------------			



	// State QUOTATION
	//				 Character is Quoted. Store directly to Buffer
	// next:
	//				 RECEIVE always
	//
    case _ASCST_QT_ : 
			abuf_writebyte(ASC_BUF_IN, S0RBUF); // read Character into Buffer
			asc_received_data++;				// One Character recieved
            if (asc_received_data > _ASC_MAX_LEN_) asc_com_state = _ASCST_WAIT_;
			asc_com_state = _ASCST_RCVD_; 		// goto State normal Communication
			break;
	// -----------------------------------------------------------------------			
    
  }
  S0RIR = 0;             // reset receive interrupt request flag
}  
//////////////////////////////////////////////////////////////////////////////


/// **************************************************************************
/// TRAMIT DATA ##############################################################
/// Using S0TBINT for asc0 transmission
/// **************************************************************************


//////////////////////////////////////////////////////////////////////////////
/// \brief send Bytes
/// send all bytes from buffer, deactivate trx irq when all data send
//////////////////////////////////////////////////////////////////////////////
void asc0_sendByte(void) {
   
   // End of message or element invalid, stop trx  
   if ( isend_buffer < csend_buffer) { 
	 S0TBUF = send_buffer[isend_buffer++];	 
   } else {
     S0TBIE  		 = 0; // disable trx buffer irq	
 	 asc_trx_running = 0; // set trx flag as "trx stopped"
   }
   S0TBIR = 0;
}


//////////////////////////////////////////////////////////////////////////////
/// 
//////////////////////////////////////////////////////////////////////////////
char putchar (char x) {
 unsigned char i = 0;

 if ( csend_buffer > 50 ) csend_buffer = 0;
 if ( isend_buffer > 50 ) isend_buffer = 0;

 csend_buffer++;
 send_buffer[csend_buffer] = x;
 asc0_sendByte();
 for (i=0; i<250; i++) ;
 return x;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// serves Trasmit Buffer IRQ (S0TBIR).
/// reads element from buffer to trasmit buffer (S0TBUF) and clears S0TBIR 
//////////////////////////////////////////////////////////////////////////////
void asc0_trxbuff_interrupt(void) interrupt 0x47 {
  strobe();
  asc0_sendByte();  
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
void asc0_trx_interrupt(void) interrupt 0x2a {
  S0TIR = 0;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief start transmission
//////////////////////////////////////////////////////////////////////////////
void asc0_start_trx(char all) {
	asc_send_all = all;

    // check buffer if really new data is available for trx
	// and no trx in progress
 	//if ( !asc_trx_running && !abuf_isempty(ASC_BUF_OUT) ) {	
	if ( !abuf_isempty(ASC_BUF_OUT) ) {
		// prepare buffer for new transmission
		// if actual Element not valid, move to next element
		// note:  one move is enough, since no empty entries in buffer are 
		//        allowed
		// note2: when buffer was empty, no move is needed, otherwise 
		//        pointer always has to be moved.
		if ( !abuf_isvalid(ASC_BUF_OUT) ) abuf_moveread(ASC_BUF_OUT);

		// now we should be able to start reading buffer
		if ( abuf_startread(ASC_BUF_OUT) ) {
		
			// start transmission (see C167 Manual 11-15 S0TBINT)
			S0TBIE 			 	= 1; // enable trx buffer irq		
			asc_trx_running 	= 1; // set trx flag as "trx running"
			
			isend_buffer 		= 0;
			csend_buffer 		= 0; 


			while ( !abuf_eom(ASC_BUF_OUT) && (csend_buffer < 250) ) {
			 send_buffer[csend_buffer++] = abuf_readbyte(ASC_BUF_OUT);
			}
			abuf_stopread(ASC_BUF_OUT); // stop reading buffer

			asc0_sendByte();	// start trx by sending first byte			
		} // else something is really wrong, no error handling since
 		// failure should be reentrent on next time
	}
}
//////////////////////////////////////////////////////////////////////////////






//@}
