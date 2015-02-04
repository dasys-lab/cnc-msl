//////////////////////////////////////////////////////////////////////////////
/// \ingroup comasc Internal communication buffer for serial communication
//@{
/// \file ais_ascbuff.h
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

#ifndef _AIS_ASCBUFF_H_
#define _AIS_ASCBUFF_H_

// Main buffer settings
#define ABF_NUMBER_OF		2		// How many Buffers do exist ?
#define ABF_READ			0		// Name Buffer for better use
#define ABF_WRITE			1		// !!! have to be serially numbered !!!

// Limits / Sizes 
#define ABF_SIZE			10		// Size of Buffers (all Buffers have the same size
#define ABF_MSGSIZE			50		// Size of Message in Buffer (all same size)




/// \brief buffer element 
typedef struct _ABF_ELEMENT_ {
	char writing;
	char reading;
	char valid;
	char msglen;
	char msgpos;
	char message[ABF_MSGSIZE];   
};


/// \brief Main Buffer structure
typedef struct _ABF_BUFFER_ {
	/// buffer Element
	struct _ABF_ELEMENT_ element[ABF_SIZE];
	/// Pointer (Index) to actual read element
	char ptr_read;
	/// Pointer (Index) to actual write element
	char ptr_write;
	/// Buffer overrun flag
	char overrrun;
};



void abuf_init(void) ;
char abuf_r_overrun(char btype);
char abuf_isoverrun(char btype) ;
void abuf_delete(char btype);
char abuf_len(char btype);
char abuf_read_msglen(char btype);
char abuf_isempty(char btype);
char abuf_isfull(char btype);
char abuf_startread(char btype);
char abuf_stopread(char btype);
char abuf_startwrite(char btype);
char abuf_stopwrite(char btype);
char abuf_eom(char btype);		 
char abuf_readbyte(char btype);
char abuf_writebyte(char btype, char data);
char abuf_isvalid(char btype);
char abuf_moveread(char btype);
char abuf_movewrite(char btype);








#endif /* _AIS_ASC0_H_ */



//@}
