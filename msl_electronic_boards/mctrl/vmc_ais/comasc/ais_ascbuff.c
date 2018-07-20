//////////////////////////////////////////////////////////////////////////////
/// \ingroup comasc Internal communication buffer for serial communication
//@{
/// \file ais_ascbuff.c
///
/// \brief 	Internal communication buffer for serial communication
///
/// \author Adam Cwientzek
///
/// \version 0.8
///
/// \date 24.10.2005
///
//////////////////////////////////////////////////////////////////////////////

#include "comasc/ais_ascbuff.h"



struct _ABF_BUFFER_ 	buffer[ABF_NUMBER_OF];


//////////////////////////////////////////////////////////////////////////////
/// \brief Buffer initialisation
//////////////////////////////////////////////////////////////////////////////
void abuf_init(void) 
{
 char ibuf  = 0;  // Counter for buffer index
 char ielem = 0;  // Counter for element index

 // Init all buffers
 for (ibuf = 0; ibuf < ABF_NUMBER_OF; ibuf++) {
	buffer[ibuf].overrrun   = 0;	// no overrun

	// Empty buffer condition
	buffer[ibuf].ptr_write  = 0;	// write index to first element
	buffer[ibuf].ptr_read   = 0;	// read index to first element

	// initialize buffer elements
	for (ielem = 0; ielem < ABF_SIZE; ielem++) {
	  buffer[ibuf].element[ielem].reading 	= 0; // no read access in progress
	  buffer[ibuf].element[ielem].writing 	= 0; // no write access in progress
	  buffer[ibuf].element[ielem].valid 	= 0; // no valid data in element
	  buffer[ibuf].element[ielem].msglen	= 0; // length of data is 0
	  buffer[ibuf].element[ielem].msgpos	= 0; // position of dataptr is 0	  
	  // Message does not have to be initialized since access is not allowed 
	  // until data valid !!

	} // for ielem
 } // for ibuf
} 
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// \brief return message len of actual read buffer element
//////////////////////////////////////////////////////////////////////////////
char abuf_read_msglen(char btype) {
 char ielem = buffer[btype].ptr_read;
	if ( (!buffer[btype].element[ielem].valid) ||	// Data not valid
		 (!buffer[btype].element[ielem].reading)  )	// Buffer not in read mode
			 return 0; // no message !! (len = 0)
 return buffer[btype].element[ielem].msglen; // length of data 
}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// \brief Reset overrun Flag
//////////////////////////////////////////////////////////////////////////////
char abuf_r_overrun(char btype) 
{
	if ( !buffer[btype].overrrun ) return 0;
	buffer[btype].overrrun = 0;
	return 1;
}
//////////////////////////////////////////////////////////////////////////////

char abuf_isoverrun(char btype) 
{
	return buffer[btype].overrrun;
}



//////////////////////////////////////////////////////////////////////////////
/// \brief Delete all entries in Buffer (empty Buffer) 
///
/// Function takes shortcuts (direct buffer access) to get a better 
/// performance since it will be usually called in time-critical situations
/// as buffer overrun.
///
void abuf_delete(char btype) 
{
  char ielem;
  // loop as long as Buffer not empty
  while ( !abuf_isempty(btype) ) {
    // Move to next avaiable Element
    abuf_moveread(btype);

	// Access Buffer direct to speed up Function !
	// Get Element index
	ielem = buffer[btype].ptr_read;
	// Mark Element as empty and not valid (=delete)
	buffer[btype].element[ielem].valid = 0;
	buffer[btype].element[ielem].msgpos = 0;
	buffer[btype].element[ielem].msglen = 0;
    buffer[btype].element[ielem].reading = 0;
  }  
}



//////////////////////////////////////////////////////////////////////////////
/// \brief Buffer Length, returns how many valid messages are in Buffer
///
char abuf_len(char btype) 
{
	int len = 0;
	char irelem = buffer[btype].ptr_read;
	char iwelem = buffer[btype].ptr_write;
	// Count Elements between read and Write
	len = buffer[btype].ptr_write - buffer[btype].ptr_read; 
	// Ring buffer: on overflow add buffer size to get buffer len
	if ( len < 0 ) len += ABF_SIZE;

	// one more element  if actual read position is also valid 
	len += ( buffer[btype].element[irelem].valid == 1); 

	// one less if actual write position not valid 
	// !Only if len was > 0 !!	
	if ( (len > 0) && (buffer[btype].element[iwelem].valid == 0) )
		len -= 1;
	
	return len; // return buffer len
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief check if Buffer is empty
///
/// \returns 1 if buffer is empty
///			 0 if more than 1 valid message in Buffer
//////////////////////////////////////////////////////////////////////////////
char abuf_isempty(char btype) 
{
	return (abuf_len(btype) == 0);
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief check if buffer is full
//////////////////////////////////////////////////////////////////////////////
char abuf_isfull(char btype) 
{
	char temp_idx = (buffer[btype].ptr_write + 1) % ABF_SIZE;
	// Full means next Situation is overrun !

	// Check overrun conditions
	// Overrun situations:
	if ( (buffer[btype].ptr_read == temp_idx) ||         // End of Buffer reached
	     (buffer[btype].element[temp_idx].valid == 1) || // Next element valid (Buffer full)
		 (buffer[btype].overrrun) )						 // Overrun
			return 1;
	
	return 0;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Check Buffer if reading posibble and start reading, mark
///        buffer element as "reading"
///
/// \return 1 if start reading successful
///			0 if reading not possible
//////////////////////////////////////////////////////////////////////////////
char abuf_startread(char btype) 
{ 	
    char ielem = buffer[btype].ptr_read;

	// START:CRITICAL SEQUENCE
	// ***********************************************************************
	// It writing active or data not valid, do not start reading !
	if ( buffer[btype].element[ielem].writing ||
		 !buffer[btype].element[ielem].valid )
	  return 0;

	// Mark Buffer as "in reading"
	buffer[btype].element[ielem].reading = 1;
	// ***********************************************************************
	// CRITICAL SEQUENCE
	buffer[btype].element[ielem].msgpos = 0;

	return 1; // start reading successful
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief stop reading, mark element as "not reading", delete data valid flag
///
/// \return 1 if stop reading successful
///			0 if reading can not be stopped (was not marked as "reading")
//////////////////////////////////////////////////////////////////////////////
char abuf_stopread(char btype) 
{
   char ielem = buffer[btype].ptr_read;
    // Buffer was not in read access, error !
	if ( !buffer[btype].element[ielem].reading )
	 return 0;

   // deactivation of reading not critical
   buffer[btype].element[ielem].reading = 0;
   // data read out and not valid any more (delete after reading)
   buffer[btype].element[ielem].valid = 0;
	buffer[btype].element[ielem].msgpos = 0;
	buffer[btype].element[ielem].msglen = 0;

   return 1; // stop reading successful
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief start writing, mark element as "writing"
///
/// \return 1 if start writing successful
///			0 if writing can not be started (was not marked as "writing")
//////////////////////////////////////////////////////////////////////////////
char abuf_startwrite(char btype) 
{
	char ielem = buffer[btype].ptr_write;

	if ( buffer[btype].overrrun ) return 0; // Do not write while overrun

	// START:CRITICAL SEQUENCE
	// ***********************************************************************
	// It reading active or data still valid (not readed), do not write !
	if ( buffer[btype].element[ielem].reading ||
		 buffer[btype].element[ielem].valid )
	  return 0;

	// Mark Buffer as "in writing"
	buffer[btype].element[ielem].writing = 1;
	// ***********************************************************************
	// CRITICAL SEQUENCE
	buffer[btype].element[ielem].msgpos = 0;
	buffer[btype].element[ielem].msglen = 0;

	return 1; // start reading successful
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief stop writing, mark element as "not writing", data now valid
///
/// \return 1 if stop writing successful
///			0 if writing can not be stopped (was not marked as "writing")
//////////////////////////////////////////////////////////////////////////////
char abuf_stopwrite(char btype) 
{
   char ielem = buffer[btype].ptr_write;
    // Buffer was not in write access, error !
	if ( !buffer[btype].element[ielem].writing )
	 return 0;

   // deactivation of writing not critical
   buffer[btype].element[ielem].writing = 0;
   // data now new and valid 
   buffer[btype].element[ielem].valid = 1;

   return 1; // stop writing successful
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief End of message, check if readposition is out of range 
///        (datapointer out of data area)
///
/// \return 1 if out of range 
///			0 if within allowed range (next element can be read/write) or
///			  reading not allowed (read flag not set)
//////////////////////////////////////////////////////////////////////////////
char abuf_eom(char btype) 
{
  char ielem = buffer[btype].ptr_read;
  if ( !buffer[btype].element[ielem].reading ) return 1;
  // If message pointer is behind message end, "end of message" is reached
  return (buffer[btype].element[ielem].msgpos >= buffer[btype].element[ielem].msglen);
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief read next byte from message buffer actual element. Move Pointer to 
///        next byte in message for next reading
///
/// \return next byte from message. 0 if index out out range or data not valid.
/// 
/// \warning Do NOT use 0 to detect end of message (use abuf_eom) since valid
///          data can also be 0 !
//////////////////////////////////////////////////////////////////////////////		 
char abuf_readbyte(char btype) 
{
    char ielem = buffer[btype].ptr_read;
	char dpos  = buffer[btype].element[ielem].msgpos; 
	
    if ( abuf_eom(btype) ) return 0; // Out of Range ! save condition = 0
	if ( (!buffer[btype].element[ielem].valid) ||	// Data not valid
		 (!buffer[btype].element[ielem].reading)  )	// Buffer not in read mode
			 return 0; // do not read

	// If within valid range, read next message byte, move pointer to next byte !
	buffer[btype].element[ielem].msgpos++;
	return buffer[btype].element[ielem].message[dpos];
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief write next byte to message buffer actual element. Move Pointer to 
///        next byte in message for next writing, increase msg length
///
/// \return 1 if writing successfull
///         0 if error (out of range ABF_MSGSIZE)
//////////////////////////////////////////////////////////////////////////////	
char abuf_writebyte(char btype, char data) 
{
    char ielem = buffer[btype].ptr_write;
	char dpos  = buffer[btype].element[ielem].msgpos; 
	
	if ( ! buffer[btype].element[ielem].writing ) return 0;

    if ( dpos >= ABF_MSGSIZE ) return 0; // Out of Range ! do not write !
		

	// If within valid range, read next message byte, move pointer to next byte !
	buffer[btype].element[ielem].msgpos++;
	buffer[btype].element[ielem].msglen++;
	buffer[btype].element[ielem].message[dpos] = data;
	return 1;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief check if Data at actual Position is valid
///
/// \return 1 if data valid
///         0 if data not valid
//////////////////////////////////////////////////////////////////////////////
char abuf_isvalid(char btype) 
{
    char ielem = buffer[btype].ptr_read;

	return buffer[btype].element[ielem].valid;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Move pointer of first ring-buffer element
///
/// Will move Pointer (index) of read Element to next buffer entry. If actual
/// element is valid (not readed yet) or if no next element exists, index
/// will not be moved.
///
/// \return 0 if index could not be moved
///         1 if index was moved 
//////////////////////////////////////////////////////////////////////////////
char abuf_moveread(char btype) 
{
	char ielem = buffer[btype].ptr_read;

	// Do not move read index if this Buffer Element is valid
	// or if no more Elements available (ptr_read = ptr_write)
	if ( (buffer[btype].ptr_write == buffer[btype].ptr_read) || 
		 ( buffer[btype].element[ielem].valid == 1) ||
		 ( buffer[btype].element[ielem].reading == 1) ) 
		return 0;

	// Move to next element
	buffer[btype].ptr_read = ++buffer[btype].ptr_read % ABF_SIZE;    

	return 1;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \return 0 if overrun !!! (has to be handled and reset!)
//////////////////////////////////////////////////////////////////////////////
char abuf_movewrite(char btype) 
{
	char ielem = buffer[btype].ptr_write;
	char temp_idx = (buffer[btype].ptr_write + 1) % ABF_SIZE;

	// Check overrun conditions

	// Move only if NOT:

	// Overrun situations:
	if ( abuf_isfull(btype) )	{	// Overrun
			//Set overrun flag
		buffer[btype].overrrun = 1; // must be reset after error handling !
		return 0;
	}

	// If Buffer in writing or not written (valid=0) do not move write pointer !
	if ( (buffer[btype].element[ielem].writing == 1) ||
		 (buffer[btype].element[ielem].valid == 0) ) return 0;

	// If next element free and no overrun situation: move to next element
	buffer[btype].ptr_write = temp_idx;    
	return 1;
}
//////////////////////////////////////////////////////////////////////////////




//@}






