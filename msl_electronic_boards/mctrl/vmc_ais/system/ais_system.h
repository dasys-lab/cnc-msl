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

#ifndef _AIS_SYSTEM_H_
#define _AIS_SYSTEM_H_

// Commands
#define _BASECMD_ERR_BUFFER_	0x02
#define _BASECMD_ERR_UNKNOWN_	0x05
#define _BASECMD_VER_			0x10 
#define _BASECMD_ECHO_			0x11 




void ais_system_init();
void ais_base_proceed(struct CmdStruct Command);
void ais_system_periodic();
void ais_system_proceed(struct CmdStruct Command);
void ais_system_bufferError();


#endif /* _AIS_SYSTEM_H_ */

//@}