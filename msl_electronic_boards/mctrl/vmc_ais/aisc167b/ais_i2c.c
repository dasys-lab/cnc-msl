//////////////////////////////////////////////////////////////////////////////
/// \defgroup 12r I^2C Interface
//@{
/// \file ais_i2c.c
///
/// \brief 	Driver Functions for Software-emulated I2C Interface \n
/// 		on the AISC167Board
///
/// \author Adam Cwientzek
///
/// \version 0.8
///
/// \date 30.06.2005
///
/// \note To Change Pins for I2C Bus edit the file ic2.h
///
//////////////////////////////////////////////////////////////////////////////

#include <reg165.h>
#include <INTRINS.H>
#include "aisc167b/ais_i2c.h"

// only for printf
#include "comasc/ais_ascbuff.h"
#include <stdio.h>


//////////////////////////////////////////////////////////////////////////////
/// Delay Function used by I2C Driver
///
/// \param count Cycles to wait
///
/// \attention 	this function wastes CPU time. Change if better performance \n
/// 			is needed.
//////////////////////////////////////////////////////////////////////////////
void i2c_delay(VMC_UINT_16 count) {
 while (count--) ;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// Wait for I2C Slave if Slave is slower that Master during communication.
/// Set the length of timeout in Header-File i2c.h ( i2c_timeout )
/// If the timeout is reached while waiting a 0 will be returned.
///
/// \return 0 if timeout \n
///			1 if slave answered within timeout
///
///
/// \attention 	this function wastes CPU time. Change if better performance \n
/// 			is needed.
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 i2c_waitDevice(void) {
 VMC_UINT_16 wait = i2c_timeout;

 DIR_SCL = 0;      // Clock as Input

 i2c_delay(2);

 while ( wait--) {	
  if ( DAT_SCL ) { // Wait for Device to "pull" CLK up
   DAT_SCL = 1;    // Set SCL to 1 (Line IS allready 1 !)
   DIR_SCL = 1;    // Set SCL as output
   return (0);     // return no Error, stop waiting.
  }
 }
 // Timeout run out
 return (1); // Error waiting for Device
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// Initialize I2C Interface. Sets I2C Pins to open-drain and input.
///
/// \return 0 if initialized \n
///			1 if initialization failed due to busy I2C bus
///
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 i2c_init(void) {
    OD_SCL  = 1;  // Set SCL Pin to open drain
	OD_SDA  = 1;  // Set SDA Pin to open drain

	DIR_SCL = 0;  // Set SCL Pin to input
	DIR_SDA = 0;  // Set SDA Pin to input

	// Check for busy Lines on Bus
	if ( !DAT_SDA || !DAT_SCL )
	  if (i2cStop()) // try to Stop Communication if Lines are busy
		return (1); // return "busy I2C Bus"

	return (0); // Return "I2C Bus Ready"
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// Start Transmission on I2C Bus.
///
/// \see I2C Specification "Start Condition on I2C Bus"
///
//////////////////////////////////////////////////////////////////////////////
void i2c_start(void) {
  OD_SCL = 1; // Set CLK-Pin to open drain
  OD_SDA = 1; // Set SDA-Pin to open drain

  DAT_SCL = 1; // Set Clock high
  DAT_SDA = 1; // Set Data high

  DIR_SCL = 1; // Set Direction Clock output
  DIR_SDA = 1; // Set Direction Data output

  i2c_delay(10); // Delay

  // Start Condition for I2C
  DAT_SDA = 0; // SDA low first
  i2c_delay(10);   // after Delay
  DAT_SCL = 0; // followed by SCL line

  i2c_delay(200);

}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// Send Byte on I2C Bus
///
/// \param data Databyte to send
///
/// \return 0 if Data was acknowledged by Slave \n
///			1 if no Slave answered
///
/// \see I2C Specification "Sending Data" and "Slave Acknowledgement"
///
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 i2c_sendByte(VMC_UCHAR_8 data) {
  VMC_UCHAR_8 mask, position;

  mask = 0x80; // Mask set to "1000 0000"
  DIR_SDA = 1; // Data is output
  DIR_SCL = 1; // Clock is output

  // now proceed all Bits of Data-Byte one by one
  for ( position=8; position>0; position--) {
   if ( mask & data ) DAT_SDA = 1;  // Send Bit
   else DAT_SDA = 0;
	
   mask = mask >> 1;  // Shift Mask to next Bit

   i2c_delay(2); // short Delay after setting Databit

   // Now wait for Device to accept send Bit
   if ( i2c_waitDevice() ) return (1);

   DIR_SCL = 1; // Clock as output again
   DAT_SCL = 0; // Prepare for next Bit
  }

  DIR_SDA = 0; // SDA now Input
  i2c_delay(3);
  DAT_SCL = 1; // Generate 9th Clock Pulse for Device ACK
  i2c_delay(5);
  position = DAT_SDA; // Read ACK bit from Device
  DAT_SCL = 0; // Set clock low again, end of byte transmission
  i2c_delay(2);

  i2c_delay(500);
  return (position);
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// Get Byte from I2C Bus and send specified Acknowledgement
///
/// \param MasterACK Acknowledgement to send (0 / 1)
///
/// \return recived Data from Bus
///
/// \see I2C Specification "Recieving Data" and "Master Acknowledgement"
///
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 i2c_getByte(VMC_UCHAR_8 MasterACK) {
 VMC_UCHAR_8 mask, position, data;

 data = 0;		// init Data-Buffer to 0
 mask = 0x80;	// init Mask to 1000 0000
 DIR_SDA = 0; 	// set SDA to Input

 // now read 8 Bits from I2C Bus
 for ( position=8; position>0; position--) {
  i2c_delay(3);
	
  if ( i2c_waitDevice() ) return (0); // Wait for Slave
  i2c_delay(2);
  if ( DAT_SDA ) data |= mask; // Read Bit from SDA Line
  mask = mask >> 1;	// Shift mask to next Bit
  DAT_SCL = 0; 		// Pull Clock to 0
  DIR_SCL = 1; 		// Output Clock Signal
 }

 if (MasterACK)  DAT_SDA = 1;  // send ACK or NOACK to Slave
 else DAT_SDA = 0;

 DIR_SDA = 1;	// Set SDA to output
 i2c_delay(2);

 DAT_SCL = 1; 	// one Clock Signal (9th for ACK)
 i2c_delay(5);
 DAT_SCL = 0; 	// end Clock Signal

 i2c_delay(2);

 i2c_delay(300);
 return (data); // Return recived Data
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// Stop Communication on I2C Bus.
///
/// \return 0 if Bus could be freed \n
///			1 if Bus is still busy
///
/// \see I2C Specification "Stop Condition"
///
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 i2cStop(void) {
    VMC_UINT_16 count = i2c_timeout;

	DIR_SDA = 0;        		// Set SDA to input

	while ( count-- )		  	// Repeat Count-Times
	{
	   if (DAT_SDA) {           // Is SDA-Line free ?
	       	DAT_SDA = 0;		// Pull SDA down
	     	DIR_SDA = 1;       	// Set SDA to output

        	if (i2c_waitDevice())   // Wait for Slave
	   	   		return (1);     	// Error if Slave pulls down Clock after
									// Timeout
       	   	i2c_delay(10);

			DAT_SDA = 1;		// Set SDA Line to 1
			return (0);			// Return 0, I2C Communication stopped.
	   } else {					// When SDA Line is free
	       	DAT_SCL = 1;        // Generate Clock Signal
			i2c_delay(8);
	    	DAT_SCL = 0;
	    	i2c_delay(8);
	  }
    }

    return (1);   // Timeout. I2C Bus could not be freed
}






//////////////////////////////////////////////////////////////////////////////

//@}
